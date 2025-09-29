import { Obstacle } from "./obstacle";
import { Position } from "./robot";


export enum AlgoType {
  EXHAUSTIVE_ASTAR = "Exhaustive Astar",
  EUCLIDEAN = "Euclidean",
  BFS = "Breadth First Search",
}

export const AlgoTypeList = [
  AlgoType.EXHAUSTIVE_ASTAR,
  AlgoType.EUCLIDEAN,
  AlgoType.BFS,
];

export interface AlgoInput {
  cat: string;
  value: {
    mode: number;
    obstacles: Obstacle[];
    initial_position?: Position;
  };
  server_mode: string;
  algo_type: string;
}
