from gymnasium.wrappers import TimeLimit
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.evaluation import evaluate_policy
from software.ml.simulator_env import SimulatorGymEnv
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.logger import Video
from stable_baselines3.common.logger import configure
import uuid
import time
import datetime
import cv2
import torch
import numpy as np
import gymnasium as gym


class VideoRecorderCallback(BaseCallback):
    def __init__(
        self,
        out_path: str,
        eval_env: gym.Env,
        render_freq: int,
        n_eval_episodes: int = 1,
        deterministic: bool = True,
    ):
        """Records a video of an agent's trajectory traversing ``eval_env`` and logs it to TensorBoard

        :param eval_env: A gym environment from which the trajectory is recorded
        :param render_freq: Render the agent's trajectory every eval_freq call of the callback.
        :param n_eval_episodes: Number of episodes to render
        :param deterministic: Whether to use deterministic or stochastic policy
        """
        super().__init__()
        self._out_path = out_path
        self._eval_env = eval_env
        self._render_freq = render_freq
        self._n_eval_episodes = n_eval_episodes
        self._deterministic = deterministic

    def _on_step(self) -> bool:
        if self.n_calls % self._render_freq == 0:
            screens = []

            def grab_screens(_locals, _globals) -> None:
                """Renders the environment in its current state, recording the screen in the captured `screens` list

                :param _locals: A dictionary containing all local variables of the callback's scope
                :param _globals: A dictionary containing all global variables of the callback's scope
                """
                # We expect `render()` to return a uint8 array with values in [0, 255] or a float array
                # with values in [0, 1], as described in
                # https://pytorch.org/docs/stable/tensorboard.html#torch.utils.tensorboard.writer.SummaryWriter.add_video
                screen = self._eval_env.render()
                # PyTorch uses CxHxW vs HxWxC gym (and tensorflow) image convention
                screens.append(screen.transpose(2, 0, 1))

            evaluate_policy(
                self.model,
                self._eval_env,
                callback=grab_screens,
                n_eval_episodes=self._n_eval_episodes,
                deterministic=self._deterministic,
            )
            self.logger.record(
                "trajectory/video",
                Video(torch.from_numpy(np.asarray([screens])), fps=10),
                exclude=("stdout", "log", "json", "csv"),
            )
            self.logger.info("Logged video to tensorboard")

            # stupid bodge to record an extra video
            obs, _ = self._eval_env.reset()
            img = self._eval_env.render()
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            video_writer = cv2.VideoWriter(
                f"{self._out_path}/step_{self.n_calls}.mp4",
                fourcc,
                10.0,
                (img.shape[1], img.shape[0]),
            )

            for i in range(1000):
                action, _states = self.model.predict(obs)
                obs, reward, done, truncated, info = self._eval_env.step(action)
                img = self._eval_env.render()
                video_writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

                if done:
                    break

            video_writer.release()
            print(f"output video saved in {self._out_path}")

        return True


def train():
    path = f"/home/jordan/thunderzero_logs/{datetime.datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H%M%S')}"
    logger = configure(path, ["stdout", "tensorboard"])
    create_sim_env = lambda: TimeLimit(
        SimulatorGymEnv(
            simulator_runtime_dir=f"{path}/thunderbots_simulator/{uuid.uuid4().__str__()[:8]}"
        ),
        max_episode_steps=1000,
    )  # set time limit to 5 real minutes per episode
    vec_env = SubprocVecEnv([create_sim_env for i in range(6)])
    video_callback = VideoRecorderCallback(
        out_path=path, eval_env=create_sim_env(), render_freq=10_000, n_eval_episodes=1
    )

    model = PPO("MlpPolicy", vec_env, verbose=1, tensorboard_log=f"{path}/tb_logs")
    model.set_logger(logger)
    model.learn(total_timesteps=10_000_000, callback=video_callback)

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
