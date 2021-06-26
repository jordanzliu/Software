#include "software/ai/hl/stp/play/defense_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/logger/logger.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/world/game_state.h"
#include "software/world/team.h"

DefensePlay::DefensePlay(std::shared_ptr<const PlayConfig> config) : Play(config, true) {}

bool DefensePlay::isApplicable(const World &world) const
{
    return (world.gameState().isPlaying() &&
            world.getTeamWithPossession() == TeamSide::ENEMY &&
            world.getTeamWithPossessionConfidence() >= 1.0) ||
           (world.gameState().isPlaying() &&
            play_config->getDefensePlayConfig()->getDefenseCheeseEnabled()->value() &&
            world.friendlyTeam().numRobots() < world.enemyTeam().numRobots());
}

bool DefensePlay::invariantHolds(const World &world) const
{
    return (world.gameState().isPlaying() &&
            world.getTeamWithPossession() == TeamSide::ENEMY &&
            world.getTeamWithPossessionConfidence() >= 1.0) ||
           (world.gameState().isPlaying() &&
            play_config->getDefensePlayConfig()->getDefenseCheeseEnabled()->value() &&
            world.friendlyTeam().numRobots() < world.enemyTeam().numRobots());
}

void DefensePlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    auto attacker_tactic =
        std::make_shared<AttackerTactic>(play_config->getAttackerTacticConfig());

    std::array<std::shared_ptr<CreaseDefenderTactic>, 3> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
    };

    std::array<std::shared_ptr<ShadowEnemyTactic>, 2> shadow_enemy_tactics = {
        std::make_shared<ShadowEnemyTactic>(),
        std::make_shared<ShadowEnemyTactic>(),
    };

    auto move_tactics = std::vector<std::shared_ptr<MoveTactic>>{
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true)};

    std::vector<std::shared_ptr<StopTactic>> stop_tactics = {
        std::make_shared<StopTactic>(false), std::make_shared<StopTactic>(false)};

    do
    {
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);
        // whether we should run 3 crease defenders
        const bool is_defense_cheesing =
            (play_config->getDefensePlayConfig()->getDefenseCheeseEnabled()->value()
             && world.friendlyTeam().numRobots() < world.enemyTeam().numRobots());
        // priority is crease defenders, shadowers
        PriorityTacticVector result = {{}, {}};

        // Update crease defenders
        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        result[0].emplace_back(std::get<0>(crease_defender_tactics));
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::RIGHT);
        result[0].emplace_back(std::get<1>(crease_defender_tactics));

        if (is_defense_cheesing)
        {
            // 3 crease defenders
            std::get<2>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::CENTRE);
            result[0].emplace_back(std::get<2>(crease_defender_tactics));
        } else {
            if (attacker_tactic->done())
            {
                attacker_tactic =
                    std::make_shared<AttackerTactic>(play_config->getAttackerTacticConfig());
            }
            result[1].emplace_back(attacker_tactic);
        }

        // Determine how many "immediate" enemy threats there are. If there is only one we
        // have both shadow enemy tactics swarm and block the "immediate" threat.
        // Otherwise we assign ShadowEnemy tactics for the next highest threats. If there
        // any extra friendly robots, have them perform a reasonable default defensive
        // tactic
        int immediate_enemy_threats = static_cast<int>(std::count_if(
            enemy_threats.begin(), enemy_threats.end(), [this, world](auto enemy_threat) {
                return distance(world.field().friendlyGoal(),
                                enemy_threat.robot.position()) <
                       play_config->getDefensePlayConfig()
                           ->getImmediateThreatDistance()
                           ->value();
            }));


        if (immediate_enemy_threats == 1)
        {
            std::get<0>(shadow_enemy_tactics)
                ->updateControlParams(enemy_threats.at(0),
                                      ROBOT_SHADOWING_DISTANCE_METERS);
            std::get<1>(shadow_enemy_tactics)
                ->updateControlParams(enemy_threats.at(0),
                                      ROBOT_SHADOWING_DISTANCE_METERS);
            result[1].insert(result[1].end(), shadow_enemy_tactics.begin(),
                             shadow_enemy_tactics.end());
        }
        else
        {
            if (enemy_threats.size() > 0)
            {
                std::get<0>(shadow_enemy_tactics)
                    ->updateControlParams(enemy_threats.at(0),
                                          ROBOT_SHADOWING_DISTANCE_METERS);
                result[1].emplace_back(std::get<0>(shadow_enemy_tactics));
            }
            else
            {
                result[1].emplace_back(move_tactics[0]);
            }

            if (enemy_threats.size() > 1)
            {
                std::get<1>(shadow_enemy_tactics)
                    ->updateControlParams(enemy_threats.at(1),
                                          ROBOT_SHADOWING_DISTANCE_METERS);
                result[1].emplace_back(std::get<1>(shadow_enemy_tactics));
            }
            else
            {
                auto nearest_enemy_robot =
                    world.enemyTeam().getNearestRobot(world.ball().position());
                if (nearest_enemy_robot)
                {
                    // Blocks in front of where the closest enemy robot is
                    Point block_point =
                        nearest_enemy_robot->position() +
                        Vector::createFromAngle(nearest_enemy_robot->orientation()) *
                            ROBOT_SHADOWING_DISTANCE_METERS;
                    move_tactics[1]->updateControlParams(
                        block_point, nearest_enemy_robot->orientation() + Angle::half(),
                        0.0);
                    result[1].emplace_back(move_tactics[1]);
                }
                else
                {
                    LOG(WARNING)
                        << "There are no enemy robots so a MoveTactic is not being assigned";
                }
            }
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, DefensePlay, PlayConfig> factory;
