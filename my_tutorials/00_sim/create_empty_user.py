import argparse
from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description="Tutorial on creating an empty stage.")
add_launcher_args(parser)

args_cli = parser.parse_args()

from isaaclab.sim import SimulationCfg, SimulationContext

def main():
  sim_cfg = SimulationCfg(dt=0.01, device=args_cli.device)
  
  with launch_simulation(sim_cfg, args_cli):
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.5,2.5,2.5], [0.0, 0.0, 0.0])
    sim.reset()
    print(f"[INFO]: Physics time step: {sim.get_physics_dt()}")
    
    print("[INFO]: Setup complete...")
    while sim.is_running():
      sim.step()
      if sim.get_physics_step_count() >= 500:
        print(f"[INFO]: Simulated {sim.get_physics_step_count()} steps.")
        break
      
if __name__ == "__main__":
  main()