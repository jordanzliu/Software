#include <gtest/gtest.h>

#include "test/test_util/test_util.h"
#include "util/refbox_constants.h"

// tuple of start state, update state, end state, our_restart, restart reason
typedef std::tuple<RefboxGameState, RefboxGameState, RefboxGameState, bool,
                   GameState::RestartReason>
    StateTransitionTestParams;

class GameStateTransitionTest : public ::testing::TestWithParam<StateTransitionTestParams>
{
};

TEST_P(GameStateTransitionTest, test_state_transitions)
{
    RefboxGameState start_state             = std::get<0>(GetParam());
    RefboxGameState update_state            = std::get<1>(GetParam());
    RefboxGameState end_state               = std::get<2>(GetParam());
    bool our_restart                        = std::get<3>(GetParam());
    GameState::RestartReason restart_reason = std::get<4>(GetParam());

    GameState game_state;
    game_state.updateRefboxGameState(start_state);
    game_state.updateRefboxGameState(update_state);
    EXPECT_EQ(game_state.getRefboxGameState(), end_state);
    EXPECT_EQ(game_state.isOurRestart(), our_restart);
    EXPECT_EQ(game_state.getRestartReason(), restart_reason);
}

#define STATE_TRANSITION_PARAMS(start, update, end, our_restart, restart_reason)         \
    std::make_tuple<RefboxGameState, RefboxGameState, RefboxGameState, bool,             \
                    GameState::RestartReason>(                                           \
        RefboxGameState::start, RefboxGameState::update, RefboxGameState::end,           \
        our_restart, GameState::RestartReason::restart_reason)

INSTANTIATE_TEST_CASE_P(
    All, GameStateTransitionTest,
    ::testing::Values(
        STATE_TRANSITION_PARAMS(HALT, STOP, STOP, false, NONE),
        STATE_TRANSITION_PARAMS(STOP, HALT, HALT, false, NONE),
        STATE_TRANSITION_PARAMS(STOP, PREPARE_KICKOFF_US, PREPARE_KICKOFF_US, true,
                                KICKOFF),
        STATE_TRANSITION_PARAMS(STOP, PREPARE_KICKOFF_THEM, PREPARE_KICKOFF_THEM, false,
                                KICKOFF),
        STATE_TRANSITION_PARAMS(STOP, PREPARE_PENALTY_US, PREPARE_PENALTY_US, true,
                                PENALTY),
        STATE_TRANSITION_PARAMS(STOP, PREPARE_PENALTY_THEM, PREPARE_PENALTY_THEM, false,
                                PENALTY),
        STATE_TRANSITION_PARAMS(STOP, DIRECT_FREE_US, DIRECT_FREE_US, true, DIRECT),
        STATE_TRANSITION_PARAMS(STOP, DIRECT_FREE_THEM, DIRECT_FREE_THEM, false, DIRECT),
        STATE_TRANSITION_PARAMS(STOP, INDIRECT_FREE_US, INDIRECT_FREE_US, true, INDIRECT),
        STATE_TRANSITION_PARAMS(STOP, INDIRECT_FREE_THEM, INDIRECT_FREE_THEM, false,
                                INDIRECT),
        STATE_TRANSITION_PARAMS(STOP, TIMEOUT_US, TIMEOUT_US, false, NONE),
        STATE_TRANSITION_PARAMS(STOP, TIMEOUT_THEM, TIMEOUT_THEM, false, NONE),
        STATE_TRANSITION_PARAMS(STOP, BALL_PLACEMENT_US, BALL_PLACEMENT_US, true,
                                BALL_PLACEMENT),
        STATE_TRANSITION_PARAMS(STOP, BALL_PLACEMENT_THEM, BALL_PLACEMENT_THEM, false,
                                BALL_PLACEMENT),
        // a restart state transitioning to NORMAL_START should not clear the restart. We
        // should do that ourselves either when the ball starts moving, or when we kick
        // the ball.
        STATE_TRANSITION_PARAMS(PREPARE_KICKOFF_US, NORMAL_START, NORMAL_START, true,
                                KICKOFF),
        STATE_TRANSITION_PARAMS(PREPARE_KICKOFF_THEM, NORMAL_START, NORMAL_START, false,
                                KICKOFF),
        STATE_TRANSITION_PARAMS(PREPARE_PENALTY_US, NORMAL_START, NORMAL_START, true,
                                PENALTY),
        STATE_TRANSITION_PARAMS(PREPARE_PENALTY_THEM, NORMAL_START, NORMAL_START, false,
                                PENALTY),
        STATE_TRANSITION_PARAMS(DIRECT_FREE_US, NORMAL_START, NORMAL_START, true, DIRECT),
        STATE_TRANSITION_PARAMS(DIRECT_FREE_THEM, NORMAL_START, NORMAL_START, false,
                                DIRECT),
        STATE_TRANSITION_PARAMS(INDIRECT_FREE_US, NORMAL_START, NORMAL_START, true,
                                INDIRECT),
        STATE_TRANSITION_PARAMS(INDIRECT_FREE_THEM, NORMAL_START, NORMAL_START, false,
                                INDIRECT),
        STATE_TRANSITION_PARAMS(NORMAL_START, GOAL_US, GOAL_US, false, NONE),
        STATE_TRANSITION_PARAMS(NORMAL_START, GOAL_THEM, GOAL_THEM, false, NONE)));

class GameStatePredicateTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        allRefboxGameStates = ::Test::TestUtil::getAllRefboxGameStates();
    }
    std::vector<RefboxGameState> allRefboxGameStates;
};

#define PREDICATE_TEST(predicate, ...)                                                   \
    TEST_F(GameStatePredicateTest, predicate##_test)                                     \
    {                                                                                    \
        std::set<RefboxGameState> true_states = {__VA_ARGS__};                           \
        for (auto refbox_game_state : allRefboxGameStates)                               \
        {                                                                                \
            GameState game_state;                                                        \
            game_state.updateRefboxGameState(refbox_game_state);                         \
            std::cout << "Testing predicate " << #predicate << " with state "            \
                      << refbox_game_state << std::endl;                                 \
            if (true_states.find(refbox_game_state) != true_states.end())                \
            {                                                                            \
                EXPECT_TRUE(game_state.predicate());                                     \
            }                                                                            \
            else                                                                         \
            {                                                                            \
                EXPECT_FALSE(game_state.predicate());                                    \
            }                                                                            \
        }                                                                                \
    }

PREDICATE_TEST(isHalted, RefboxGameState::HALT, RefboxGameState::TIMEOUT_US,
               RefboxGameState::TIMEOUT_THEM)

// testing state transitions
// parameterize over a tuple of start state, update state, end state, our_restart,
// restart_reason? test for end state and restart reason

// paramaterized test over refboxgamestates
// test all predicates


int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
};