#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"
#include "State_Handover.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
    // The SDK default (1000 ms) holds the last motor targets for a full second
    // after DDS drops. LowState arrives at ~500 Hz, so 100 ms is ~50 lost messages.
    // Set after connecting so the startup wait loop is unchanged.
    FSMState::lowstate->set_timeout_ms(100);
    spdlog::info("LowState timeout set to 100 ms.");
}

// --domain N picks the DDS domain: 0 = real robot, 1 = unitree_mujoco
// (simulate/config.yaml). The shared param::helper (deploy/include/param.h)
// rejects unknown options, so it is taken out of argv here first.
// Returns -1 when --domain is not given (main() then picks it from --network).
static int take_domain_arg(int& argc, char** argv)
{
    int domain = -1;
    int out = 1;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        std::string value;
        if (a == "--domain" && i + 1 < argc) value = argv[++i];
        else if (a.rfind("--domain=", 0) == 0) value = a.substr(9);
        else { argv[out++] = argv[i]; continue; }
        try {
            size_t used = 0;
            domain = std::stoi(value, &used);
            if (used != value.size() || domain < 0) throw std::invalid_argument(value);
        } catch (const std::exception&) {
            std::cerr << "Bad --domain '" << value << "' (0 = real robot, 1 = sim)" << std::endl;
            std::_Exit(1);  // exit() would trip the static keyboard thread (std::terminate)
        }
    }
    argc = out;
    argv[argc] = nullptr;
    return domain;
}

int main(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h")
            std::cout << "  --domain arg          DDS domain (default: 1 on lo = sim, 0 otherwise = real robot)\n";
    }
    int domain = take_domain_arg(argc, argv);

    // Load parameters
    auto vm = param::helper(argc, argv);
    // Sim is the default: no --network means unitree_mujoco on lo, domain 1.
    // The real robot needs an explicit --network enp43s0 (domain then defaults to 0).
    // (An empty interface would let the SDK pick one itself, possibly enp43s0.)
    std::string network = vm["network"].as<std::string>();
    if (network.empty()) network = "lo";
    if (domain < 0) domain = (network == "lo") ? 1 : 0;
    FSMState::keyboard_fsm_shortcuts_enabled = (network == "lo");

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1-23dof Controller \n";
    std::cout << "DDS: domain " << domain << " on " << network
              << (network == "lo" ? " (SIM, keyboard on)" : " (REAL ROBOT, keyboard off, remote only)") << "\n";
    if (network == "lo" && domain == 0)
        spdlog::warn("unitree_mujoco runs on domain 1 (simulate/config.yaml); domain 0 on lo finds no sim.");
    if (network != "lo" && domain != 0)
        spdlog::warn("The real robot uses domain 0; domain {} will not find it.", domain);

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(domain, network);

    init_fsm_state();

    FSMState::lowcmd->msg_.mode_machine() = 4; // 23dof
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }
    
    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    // Simulation keys for Passive -> Handover ([h]) and back ([p]); the shared
    // keyboard shortcuts only know u / v / p. Loopback only, like those.
    if (FSMState::keyboard_fsm_shortcuts_enabled && FSMStringMap.right.count("Handover")) {
        int passive = FSMStringMap.right.at("Passive");
        int handover = FSMStringMap.right.at("Handover");
        auto key_is = [](std::string k) {
            return [k]()->bool {
                return FSMState::keyboard && FSMState::keyboard->on_pressed &&
                       FSMState::keyboard->key() == k;
            };
        };
        for (auto & s : fsm->states) {
            if (s->isState(passive)) s->registered_checks.emplace_back(key_is("h"), handover);
            if (s->isState(handover)) s->registered_checks.emplace_back(key_is("p"), passive);
        }
    }
    fsm->start();

    std::cout << "Press [L2 + Up] to enter FixStand mode.\n";
    std::cout << "And then press [R2 + A] to start controlling the robot.\n";
    std::cout << "And then press [R1 + A/B/Y/X] to control the robot dance.\n";
    std::cout << "In Passive, hold [L1 + R1] 1 s then [START] to hand over to the Unitree controller (hung only).\n";
    if (FSMState::keyboard_fsm_shortcuts_enabled) {
        std::cout << "Simulation keyboard: [u] FixStand, [v] Velocity, [p] Passive, [h] Handover (from Passive).\n";
        std::cout << "Keyboard FSM shortcuts are enabled only with --network=lo.\n";
        std::cout << "Startup: Pause MuJoCo, Load key 0 (stand_ready), press [u] then [v] here, then Run MuJoCo.\n";
        std::cout << "Velocity keys: [w/s/a/d] move, [q/e] turn, [x/space] zero velocity. Commands stay active until changed.\n";
    }

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}

