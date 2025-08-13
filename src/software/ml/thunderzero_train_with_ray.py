import torch
from ray.rllib.algorithms.ppo import PPOConfig
from ray.tune.registry import register_env
from ray.rllib.callbacks.callbacks import RLlibCallback
from gymnasium.wrappers import TimeLimit
from software.ml.simulator_env import SimulatorGymEnv
from pprint import pprint
import uuid
import time
import datetime
import cv2
import ray


def create_env(config):
    return TimeLimit(
        SimulatorGymEnv(
            simulator_runtime_dir=f"{config.get('sim_path', '/tmp')}/thunderbots_simulator/{uuid.uuid4().__str__()[:8]}"
        ),
        max_episode_steps=300,
    )

def record_video(module, path, filename):
    # Final video recording
    env = SimulatorGymEnv(f"{path}/sim_rundir")
    obs, _ = env.reset()
    img = env.render()
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = cv2.VideoWriter(
        f"{path}/training_video.mp4", fourcc, 10.0, (img.shape[1], img.shape[0])
    )
    action_dist_class = module.get_inference_action_dist_cls()

    for i in range(1000):
        fwd_ins = {"obs": torch.Tensor([obs])}
        fwd_outputs = module.forward_exploration(fwd_ins)
        # This can be either deterministic or stochastic distribution.
        action_dist = action_dist_class.from_logits(
            fwd_outputs["action_dist_inputs"]
        )
        action = action_dist.sample()[0].numpy()
        obs, reward, done, truncated, info = env.step(action)
        img = env.render()
        video_writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        if done:
            break

    video_writer.release()
    print(f"video saved in {path}")


def train():
    path = f"/home/jordan/thunderzero_logs/{datetime.datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H%M%S')}"
    register_env("thunderzero_env", create_env)
    config = (
        PPOConfig()
        .environment(env="thunderzero_env", env_config={"sim_path": path})
        .framework("torch")
        .env_runners(num_env_runners=12, num_envs_per_env_runner=1)
        .resources(
            num_gpus=1,
        )
    )
    algo = config.build_algo()
    checkpoint_dir = None
    for i in range(10_000_000 // config.train_batch_size):
        result = algo.train()
        pprint(result)
        print(f"lifetime env steps: {result['num_env_steps_sampled_lifetime']}")
        if i % 100 == 0:
            checkpoint_dir = algo.save_to_path(f"{path}/checkpoints/step_{i}")
            record_video(algo.get_module(), f"{path}/videos", f"step_{i}.mp4")

    print(f"final checkpoint in {checkpoint_dir}")

    ray.shutdown()


if __name__ == "__main__":
    train()
