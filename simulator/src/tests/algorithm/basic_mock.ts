import { AlgoTestDataInterface } from ".";
import { Obstacle, ObstacleDirection } from "../../schemas/obstacle";

const obstacles: Obstacle[] = [
  { id: 1, x: 15, y: 10, d: ObstacleDirection.W },
  { id: 2, x: 1, y: 18, d: ObstacleDirection.S },
  { id: 3, x: 4, y: 8, d: ObstacleDirection.N },
  { id: 4, x: 5, y: 18, d: ObstacleDirection.E },
  { id: 5, x: 10, y: 2, d: ObstacleDirection.N },
];

export const AlgoTestBasicMock: AlgoTestDataInterface = {
  obstacles: obstacles,
};
