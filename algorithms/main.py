from arena.map import Map
from arena.obstacle import Obstacle

from common.consts import SNAP_COORD
from common.types import Position
from common.utils import _mappings as Int_to_Direction_mappings

from math import pi

from path_finding.hamiltonian_path import HamiltonianSearch, AlgoType

from robot.stm_commands import convert_segments_to_commands

import multiprocessing as mp
import time

""" -------------------------------------- """
""" ---------- Endpoint Schemas ---------- """
""" -------------------------------------- """
from pydantic import BaseModel
from enum import Enum
from typing import Optional

# Input
class AlgoInputMode(Enum):
  SIMULATOR = "simulator"
  LIVE = "live"

class AlgoInputValueObstacle(BaseModel):
  id: int # obstacle_id
  x: int # grid_format
  y: int # grid_format
  d: int # direction of obstacle; 1: North; 2: South; 3: East; 4: West

class AlgoInputValueInitialPosition(BaseModel):
    x: int
    y: int
    theta: float

class AlgoInputValue(BaseModel):
  obstacles: list[AlgoInputValueObstacle]
  mode: int # (0: Task 1); (1: Task 2)
  initial_position: Optional[AlgoInputValueInitialPosition] = None  # <-- Add this

class AlgoInput(BaseModel):
  cat: str = "obstacles"
  value: AlgoInputValue
  server_mode: Optional[AlgoInputMode] = AlgoInputMode.LIVE
  algo_type: Optional[AlgoType] = AlgoType.EXHAUSTIVE_ASTAR

# Output
class AlgoOutputSimulatorPosition(BaseModel):
  x: int # in cm
  y: int # in cm
  theta: float # in radian


class AlgoOutputSimulator(BaseModel):
  positions: list[AlgoOutputSimulatorPosition]
  runtime: str

class AlgoOutputLivePosition(BaseModel):
  x: int # in cm
  y: int # in cm
  d: str # Robot Face -> 1: North; 2: South; 3: East; 4: West


class AlgoOutputLiveCommand(BaseModel):
  #cat: str = "control"
  value: str
  end_position: AlgoOutputLivePosition

class AlgoOutputLive(BaseModel):
  commands: list[AlgoOutputLiveCommand]


""" -------------------------------------- """
""" ----------- Main Functions ----------- """
""" -------------------------------------- """
if __name__ == '__main__':
  mp.freeze_support() # Needed to run child processes (multiprocessing)

def main(algo_input: AlgoInput):
  # Algorithm Server Mode -> 'simulator' or 'live'
  algo_server_mode = algo_input["server_mode"]

  # Algorithm Task Mode -> 0: Task 1; 1: Task 2
  algo_task_mode = algo_input["value"]["mode"]

  # Obstacles
  obstacles = _extract_obstacles_from_input(algo_input["value"]["obstacles"], algo_server_mode)

  # Start Position
  initial_position = algo_input["value"].get("initial_position", None)
  if initial_position is None:
      start_x, start_y, start_theta = 0, 0, 1.5708
  else:
      start_x = initial_position.get("x", 0) * 10
      start_y = initial_position.get("y", 0) * 10
      start_theta = initial_position.get("theta", 1.5708)
  start_position = Position(x=start_x, y=start_y, theta=start_theta)  
  print("Start position:", start_position)

  # Map
  map = Map(obstacles=obstacles)
  #print("Map:", type(map))
  #print("Initial:", initial_position)
  #print("Start:", start_position)

  # Algorithm
  algo_type = algo_input["algo_type"]
  print("Algorithm: ", algo_type)
  algo = HamiltonianSearch(map=map, src=start_position, algo_type=algo_type)

  # Algorithm Search⭐
  min_perm, paths = algo.search()

  # Results
  if algo_server_mode == AlgoInputMode.SIMULATOR:
    simulator_algo_output = [] # Array of positions
    for path in paths:
        for node in path:
            # Convert Position to dict
            simulator_algo_output.append({
                "x": node.pos.x,
                "y": node.pos.y,
                "theta": node.pos.theta
            })
        # Position configuration to represent scanning (*only for simulator)
        simulator_algo_output.append({"x": -1, "y": -1, "theta": -2})
        simulator_algo_output.append({"x": -1, "y": -1, "theta": -1})
        # TEST
        current_perm = 1
        stm_commands = []
        commands = convert_segments_to_commands(path)
        stm_commands.extend(commands)
        stm_commands.append([f"SNAP{min_perm[current_perm]}", commands[-1][1]])
        algoOutputLiveCommands: list[AlgoOutputLiveCommand] = [] # Array of commands
        for command in stm_commands:
          algoOutputLiveCommands.append(AlgoOutputLiveCommand(
            cat="control",
            value=command[0],
            #value='test',
            #end_position='test'
            end_position=command[1]
          ))
        print(algoOutputLiveCommands)
    return simulator_algo_output
  
  
  if algo_server_mode == AlgoInputMode.LIVE:
    #print("TEST")
    current_perm = 1
    stm_commands = []

    for path in paths:
      commands = convert_segments_to_commands(path)
      stm_commands.extend(commands)

      # Add SNAP1 command after each path (from one obstacle to another) (For Raspberry Pi Team to know when to scan the image)
      stm_commands.append([f"SNAP{min_perm[current_perm]}", commands[-1][1]])
      current_perm += 1 # Increment by current_perm to access the next obstacle_id
      
    
    algoOutputLiveCommands: list[AlgoOutputLiveCommand] = [] # Array of commands
    for command in stm_commands:
      algoOutputLiveCommands.append(AlgoOutputLiveCommand(
        cat="control",
        value=command[0],
        end_position=command[1]
      ))
    
    # Add FIN as the last command (For Raspberry Pi Team to know that the algorithm has ended)
    algoOutputLiveCommands.append(AlgoOutputLiveCommand(
      cat="control",
      value="FIN",
      end_position=algoOutputLiveCommands[-1].end_position
    ))
    
    print("Commands:", algoOutputLiveCommands)

    return algoOutputLiveCommands

def _extract_obstacles_from_input(input_obstacles, algo_server_mode):
  """
  Helper function to convert input obstacles to `Obstacle` object accepted by the algorithm
  """
  obstacles = []

  grid_pos_to_c_pos_multiplier = SNAP_COORD
  if algo_server_mode == AlgoInputMode.LIVE:
    # Live mode uses 10cm grid format (so need to *2 to align with algo's 5cm grid format)
    grid_pos_to_c_pos_multiplier *= 2

  for obstacle in input_obstacles:
    obstacles.append(Obstacle(
      x=obstacle["x"] * grid_pos_to_c_pos_multiplier,
      y=obstacle["y"] * grid_pos_to_c_pos_multiplier,
      facing=Int_to_Direction_mappings[str(obstacle["d"])]
    ))

  return obstacles


""" -------------------------------------- """
""" ------ FastAPI (API Endpoints) ------- """
""" -------------------------------------- """
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

app = FastAPI()

origins = [
    "http://localhost",
    "http://localhost:3000",
    "http://10.91.175.30:3000"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
  return { "message": "Hello World" }

@app.get("/algo/simulator/simple-test", tags=["Algorithm"])
async def algo_simulator_test():
  """To test algo and endpoint on the server without starting up the web simulator"""
  # Basic Mock Data
  simulator_algo_input: AlgoInput = {
    "cat": "obstacles",
    "value": {
      "obstacles": [
        { "id": 1, "x": 30, "y": 20, "d": 4 }, # 5cm grid (x, y)
        { "id": 2, "x": 2, "y": 36, "d": 2 }, # 5cm grid (x, y)
      ],
      "mode": 0 # Task 1
    },
    "server_mode": AlgoInputMode.SIMULATOR,
    "algo_type": AlgoType.EXHAUSTIVE_ASTAR,
  }

  positions = main(simulator_algo_input)
  
  return { "positions": positions }

@app.post("/algo/simulator", response_model=AlgoOutputSimulator, tags=["Algorithm"])
async def algo_simulator(request: Request):
    data = await request.json()
    value = data.get("value", {})
    obstacles = value.get("obstacles", [])
    initial_position = value.get("initial_position", None)

    # Default start if not provided
    if initial_position is None:
        start_x, start_y, start_theta = 0, 0, 1.5708  # Facing North (radians)
    else:
        start_x = initial_position.get("x", 0)
        start_y = initial_position.get("y", 0)
        start_theta = initial_position.get("theta", 1.5708)

    # Pass these to your main logic
    algo_input = {
        "value": {
            "obstacles": obstacles,
            "mode": value.get("mode", 0),
            "initial_position": {
                "x": start_x,
                "y": start_y,
                "theta": start_theta
            }
        },
        "server_mode": AlgoInputMode.SIMULATOR,
        "algo_type": data.get("algo_type", AlgoType.EXHAUSTIVE_ASTAR)
    }
    positions = main(algo_input)
    return { "positions": positions, "runtime": "0.01s" }

@app.get("/algo/live/simple-test", response_model=AlgoOutputLive, tags=["Algorithm"])
async def algo_live_test():
  """To test algo and endpoint on the server in live mode"""
  # Basic Mock Data
  live_algo_input: AlgoInput = {
    "cat": "obstacles",
    "value": {
      "obstacles": [
        { "id": 1, "x": 15, "y": 10, "d": 4 }, # 10cm grid (x, y)
        { "id": 2, "x": 1, "y": 18, "d": 2 }, # 10cm grid (x, y)
      ],
      "mode": 0 # Task 1
    },
    "server_mode": AlgoInputMode.LIVE,
    "algo_type": AlgoType.EXHAUSTIVE_ASTAR,
  }
  commands = main(live_algo_input)
  
  return { "commands": commands }

@app.post("/algo/live", response_model=AlgoOutputLive, tags=["Algorithm"])
async def algo_live(algo_input: AlgoInput):
  """Main endpoint for live mode"""
  if hasattr(algo_input, "model_dump"):
     algo_input = algo_input.model_dump()
  else:
     algo_input = algo_input.dict()
  commands = main(algo_input)

  return { "commands": commands }
