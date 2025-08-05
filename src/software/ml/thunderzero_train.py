from gymnasium.wrappers import TimeLimit
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from software.ml.simulator_env import SimulatorGymEnv
import uuid
import time
import datetime
import cv2


def train():
    path = (
        f"/tmp/{datetime.datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H%M%S')}"
    )
    create_sim_env = lambda: TimeLimit(
        SimulatorGymEnv(
            simulator_runtime_dir=f"{path}/thunderbots_simulator/{uuid.uuid4().__str__()[:8]}"
        ),
        max_episode_steps=300,
    )  # set time limit to 5 real minutes per episode
    vec_env = make_vec_env(create_sim_env, n_envs=6)
    model = PPO(
        "MlpPolicy", vec_env, verbose=1, device="cpu", tensorboard_log=f"{path}/tb_logs"
    )
    model.learn(total_timesteps=10_000)
    model.save(f"{path}/thunderzero_model.ckpt")

    # do one rollout for rendering
    env = SimulatorGymEnv(f"{path}/sim_rundir")
    obs, _ = env.reset()
    img = env.render()
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = cv2.VideoWriter(
        f"{path}/training_video.mp4", fourcc, 10.0, (img.shape[1], img.shape[0])
    )

    for i in range(1000):
        action, _states = model.predict(obs)
        obs, reward, done, truncated, info = env.step(action)
        img = env.render()
        video_writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        if done:
            break

    video_writer.release()
    print(f"output video saved in {path}")


if __name__ == "__main__":
    train()
