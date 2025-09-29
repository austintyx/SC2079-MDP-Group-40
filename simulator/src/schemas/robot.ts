/** Position of Robot on the navigational area */
export interface Position {
  x: number;
  y: number;
  theta: number; // in Radian;
}

export function thetaToRobotDirection(theta: number): RobotDirection {
  // Normalize theta to range -Math.PI to Math.PI
  const normalizedTheta = ((theta + Math.PI) % (2 * Math.PI)) - Math.PI;

  if (normalizedTheta >= -0.7 && normalizedTheta <= 0.7) {
    return RobotDirection.W;
  } else if (normalizedTheta > 0.7 && normalizedTheta <= 2.0) {
    return RobotDirection.N;
  } else if (normalizedTheta >= -2.0 && normalizedTheta < -0.7) {
    return RobotDirection.S;
  } else {
    return RobotDirection.E;
  }
}

const directionToTheta: Record<'N' | 'S' | 'E' | 'W', number> = {
  N: 1.57,
  S: -1.57,
  E: 0,
  W: 3.14,
};

// Only accepts 'N', 'S', 'E', or 'W' as input
export function convertDirectionToTheta(direction: 'N' | 'S' | 'E' | 'W'): number {
  return directionToTheta[direction];
}





/** Simplified direction (instead of theta) that the Robot is facing. */
export enum RobotDirection {
  N = "North",
  S = "South",
  E = "East",
  W = "West",
}

/** Robot's Turn Direction when turning in an arc */
export enum TurnDirection {
  Clockwise = "Clockwise",
  Anticlockwise = "Anti-Clockwise",
}

/** Robot's available Action types */
export enum RobotActionEnum {
  Scan = "Scan",
  MoveStraight = "MoveStraight",
  CurveLeft = "CurveLeft",
  CurveRight = "CurveRight",
  MoveBack = "MoveBack",
}

/** Roboto's Action with distance and theta values */
export interface RobotAction {
  type: RobotActionEnum;
  distance_straight?: number; // in cm
  distance_arc?: number; // in cm
  theta?: number; // in radian
  turn_direction?: TurnDirection;
}
