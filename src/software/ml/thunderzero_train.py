from stable_baselines3 import PPO
from software.ml.simulator_env import SimulatorGymEnv
import uuid


def train():
    env = SimulatorGymEnv(
        simulator_runtime_dir=f"/tmp/{uuid.uuid4()}/thunderbots_simulator"
    )
    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=1_000_000)
    model.save("thunderzero_model")

    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, terminated, info = env.step(action)


if __name__ == "__main__":
    train()
