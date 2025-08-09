from ray.rllib.algorithms.ppo import PPOConfig
from ray.tune.registry import register_env
from ray.rllib.callbacks.callbacks import RLlibCallback
from gymnasium.wrappers import TimeLimit
from software.ml.simulator_env import SimulatorGymEnv
import uuid
import time
import datetime
import cv2
import ray


class VideoRecorderCallback(RLlibCallback):
    def __init__(self, out_path: str, render_freq: int = 100_000):
        self.out_path = out_path
        self.render_freq = render_freq
        self.step_count = 0

    def _record_video(self):
        env = TimeLimit(
            SimulatorGymEnv(
                simulator_runtime_dir=f"{self.out_path}/thunderbots_simulator/{uuid.uuid4().__str__()[:8]}"
            ),
            max_episode_steps=300,
        )

        # Load the latest checkpoint
        checkpoint_path = f"{self.out_path}/ray_results"

        obs, _ = env.reset()
        img = env.render()
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video_writer = cv2.VideoWriter(
            f"{self.out_path}/step_{self.step_count}.mp4",
            fourcc,
            10.0,
            (img.shape[1], img.shape[0]),
        )

        for i in range(1000):
            # For now, take random actions - would need policy for actual predictions
            action = env.action_space.sample()
            obs, reward, done, truncated, info = env.step(action)
            img = env.render()
            video_writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

            if done:
                break

        video_writer.release()
        print(f"Video saved: {self.out_path}/step_{self.step_count}.mp4")


def create_env(config):
    return TimeLimit(
        SimulatorGymEnv(
            simulator_runtime_dir=f"{config.get('sim_path', '/tmp')}/thunderbots_simulator/{uuid.uuid4().__str__()[:8]}"
        ),
        max_episode_steps=300,
    )


def train():
    path = f"/home/jordan/thunderzero_logs/{datetime.datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H%M%S')}"
    # Create callback for video recording
    # TODO: deal with callback
    video_callback = VideoRecorderCallback(out_path=path, render_freq=100_000)
    register_env("thunderzero_env", create_env)
    config = (
        PPOConfig()
        .environment(env="thunderzero_env", env_config={"sim_path": path})
        .framework("torch")
        .env_runners(num_env_runners=12, num_envs_per_env_runner=1)
        .training(num_epochs=10, lr=5e-5, train_batch_size_per_learner=1000)
        .resources(
            num_gpus=1,
        )
    )
    algo = config.build_algo()
    for i in range(1000):
        result = algo.train()
        print(f"lifetime env steps: {result['num_env_steps_sampled_lifetime']}")

    # Final video recording
    env = SimulatorGymEnv(f"{path}/sim_rundir")
    obs, _ = env.reset()
    img = env.render()
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = cv2.VideoWriter(
        f"{path}/training_video.mp4", fourcc, 10.0, (img.shape[1], img.shape[0])
    )

    for i in range(1000):
        action = env.action_space.sample()  # Would use trained policy here
        obs, reward, done, truncated, info = env.step(action)
        img = env.render()
        video_writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        if done:
            break

    video_writer.release()
    print(f"Final video saved in {path}")

    ray.shutdown()


if __name__ == "__main__":
    train()
