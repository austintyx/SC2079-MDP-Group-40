import React, { useEffect, useState } from "react";
import { NavigationGrid } from "./NavigationGrid";
import { CoreContainter } from "../CoreContainter";
import { Position } from "../../../schemas/robot";
import {
  ALGO_GRID_BLOCK_SIZE_MULTIPLIER,
  GRID_ANIMATION_SPEED,
  ROBOT_INITIAL_POSITION,
  GRID_TOTAL_WIDTH,
  GRID_TOTAL_HEIGHT,
} from "../../../constants";
import {
  FaChevronLeft,
  FaChevronRight,
  FaPause,
  FaPlay,
  FaSitemap,
  FaSpinner,
} from "react-icons/fa";
import { convertAlgoOutputToStepwisePosition } from "./utils/path_animation";
import {
  AlgoTestDataInterface,
  AlgoTestEnum,
  AlgoTestEnumMapper,
} from "../../../tests/algorithm";
import { Button } from "../../common";
import toast from "react-hot-toast";
import { TestSelector } from "./TestSelector";
import { ServerStatus } from "./ServerStatus";
import useFetch from "../../../hooks/useFetch";
import { AlgoInput, AlgoType } from "../../../schemas/algo_input";
import { AlgoOutput } from "../../../schemas/algo_output";
import { AlgorithmSelector } from "./AlgorithmSelector";

// Add this above your component
const START_POSITIONS = [
  // Bottom Left
  { label: "Bottom Left (North)", x: 0, y: 0, theta: Math.PI / 2 },
  { label: "Bottom Left (East)", x: 0, y: 2, theta: 0 },
  // Bottom Right
  { label: "Bottom Right (North)", x: GRID_TOTAL_WIDTH - 3, y: 0, theta: Math.PI / 2 },
  { label: "Bottom Right (West)", x: GRID_TOTAL_WIDTH - 1, y: 0, theta: Math.PI },
  // Top Left
  { label: "Top Left (South)", x: 2, y: GRID_TOTAL_HEIGHT - 1, theta: - Math.PI / 2},
  { label: "Top Left (East)", x: 0, y: GRID_TOTAL_HEIGHT - 1, theta: 0 },
  // Top Right
  { label: "Top Right (South)", x: GRID_TOTAL_WIDTH - 1, y: GRID_TOTAL_HEIGHT - 1, theta: - Math.PI / 2 },
  { label: "Top Right (West)", x: GRID_TOTAL_WIDTH - 1, y: GRID_TOTAL_HEIGHT - 3, theta: Math.PI },
];

export const AlgorithmCore = () => {
  const fetch = useFetch();

  // Robot's Positions
  const [robotPositions, setRobotPositions] = useState<Position[]>();
  const totalSteps = robotPositions?.length ?? 0;

  // Select Algorithm
  const [selectedAlgoTypeEnum, setSelectedAlgoTypeEnum] = useState<AlgoType>(
    AlgoType.EXHAUSTIVE_ASTAR
  );

  // Algorithm Runtime
  const [algoRuntime, setAlgoRuntime] = useState<string>("");

  // Select Tests
  const [selectedTestEnum, setSelectedTestEnum] = useState<AlgoTestEnum>(
    AlgoTestEnum.Basic_Mock
  );
  const [selectedTest, setSelectedTest] = useState<AlgoTestDataInterface>(
    AlgoTestEnumMapper[AlgoTestEnum.Basic_Mock]
  );

  // Select Tests
  useEffect(() => {
    const selectedTest = AlgoTestEnumMapper[selectedTestEnum];
    setSelectedTest(selectedTest);

    setCurrentStep(0);
    setCurrentRobotPosition(ROBOT_INITIAL_POSITION);
    setRobotPositions(undefined);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedTestEnum]);

  // Run Algorithm
  const [isAlgorithmLoading, setIsAlgorithmLoading] = useState(false);

  // Run Algorithm
  const handleRunAlgorithm = async () => {
    if (startAnimation === true || isAlgorithmLoading === true) return;
    setIsAlgorithmLoading(true);
    setAlgoRuntime("");

    // Use custom start position for Custom test, otherwise default
    const initialPosition =
      selectedTestEnum === AlgoTestEnum.Custom
        ? { x: startX, y: startY, theta: startTheta }
        : ROBOT_INITIAL_POSITION;
    console.log("Using Initial Position:", initialPosition);  

    const algoInput: AlgoInput = {
      cat: "obstacles",
      value: {
        mode: 0,
        obstacles: selectedTest.obstacles.map((o) => {
          return {
            id: o.id,
            x: o.x * ALGO_GRID_BLOCK_SIZE_MULTIPLIER,
            y: o.y * ALGO_GRID_BLOCK_SIZE_MULTIPLIER,
            d: o.d,
          };
        }),
        initial_position: initialPosition, // <-- Add this line
      },
      server_mode: "simulator",
      algo_type: selectedAlgoTypeEnum,
    };

    try {
      const algoOutput: AlgoOutput = await fetch.post(
        "/algo/simulator",
        algoInput
      );

      // Set the robot's positions for animation/steps
      setRobotPositions(convertAlgoOutputToStepwisePosition(algoOutput.positions));

      // Set the robot's current position to the first position in the response
      if (algoOutput.positions && algoOutput.positions.length > 0) {
        setCurrentRobotPosition(algoOutput.positions[0]);
      }

      setCurrentStep(0);
      setAlgoRuntime(algoOutput.runtime);
      toast.success("Algorithm ran successfully.");
    } catch (e) {
      toast.error("Failed to run algorithm. Server Error: " + e);
    }

    setIsAlgorithmLoading(false);
  };

  // Animation
  const [isManualAnimation, setIsManualAnimation] = useState(false);
  const [startAnimation, setStartAnimation] = useState(false);
  const [currentStep, setCurrentStep] = useState(0);
  const [currentRobotPosition, setCurrentRobotPosition] = useState<Position>();

  // Animation
  useEffect(() => {
    if (robotPositions && startAnimation && currentStep + 1 < totalSteps) {
      const timer = setTimeout(() => {
        const nextStep = currentStep + 1;
        setCurrentStep(nextStep);

        // Handle Scan Animation
        if (
          robotPositions[nextStep].x === -1 &&
          robotPositions[nextStep].y === -1
        ) {
          if (robotPositions[nextStep].theta === -1)
            toast.success("Image Scanned!");
          else toast("Scanning image...");
          // Do NOT update currentRobotPosition, keep previous valid position
        } else {
          setCurrentRobotPosition(robotPositions[nextStep]);
        }

        // Stop Animation at the last step
        if (nextStep === totalSteps - 1) {
          setStartAnimation(false);
        }
      }, GRID_ANIMATION_SPEED);
      return () => clearTimeout(timer);
    } else if (
      robotPositions &&
      isManualAnimation &&
      currentStep < totalSteps
    ) {
      // User manually click through the steps
      // Handle Scan Animation
      if (
        robotPositions[currentStep].x === -1 &&
        robotPositions[currentStep].y === -1
      ) {
        if (robotPositions[currentStep].theta === -1)
          toast.success("Image Scanned!");
        else toast("Scanning image...");
      } else {
        setCurrentRobotPosition(robotPositions[currentStep]);
      }
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [currentStep, totalSteps, startAnimation, isManualAnimation]);

  // Starting Position
  const [startX, setStartX] = useState(START_POSITIONS[0].x);
  const [startY, setStartY] = useState(START_POSITIONS[0].y);
  const [startTheta, setStartTheta] = useState(START_POSITIONS[0].theta);

  // Update currentRobotPosition when startX, startY, or startTheta changes for Custom test
  useEffect(() => {
    if (selectedTestEnum === AlgoTestEnum.Custom) {
      setCurrentRobotPosition({
        x: startX,
        y: startY,
        theta: startTheta,
      });
    }
  }, [startX, startY, startTheta, selectedTestEnum]);

  return (
    <CoreContainter title="Algorithm Simulator">
      {/* Server Status */}
      <ServerStatus />

      {/* Select Tests */}
      <TestSelector
        selectedTestEnum={selectedTestEnum}
        setSelectedTestEnum={setSelectedTestEnum}
        selectedTest={selectedTest}
        setSelectedTest={setSelectedTest}
        setAlgoRuntime={setAlgoRuntime}
      />

      {/* Select ALgorithm */}
      <AlgorithmSelector
        selectedAlgoTypeEnum={selectedAlgoTypeEnum}
        setSelectedAlgoTypeEnum={setSelectedAlgoTypeEnum}
        setAlgoRuntime={setAlgoRuntime}
        setRobotPositions={setRobotPositions}
      />

      {/* Run Algo */}
      <div className="mb-4 flex justify-center">
        <Button onClick={handleRunAlgorithm}>
          <span>Run Algorithm</span>
          {isAlgorithmLoading ? (
            <FaSpinner className="animate-spin" />
          ) : (
            <FaSitemap className="text-[18px]" />
          )}
        </Button>
      </div>

      {/* Algo Runtime */}
      {algoRuntime && (
        <div className="mb-4 flex justify-center">
          Algorithm Runtime:&nbsp;
          <span className="font-bold">{algoRuntime}</span>&nbsp;(
          {selectedAlgoTypeEnum})
        </div>
      )}

      {/* Animation */}
      {robotPositions && (
        <div className="mt-2 mb-4 flex flex-col justify-center items-center gap-2">
          {/* Position Info */}
          <div className="text-sm font-mono mb-2">
            <span className="font-bold">Current Position:</span>{" "}
            {robotPositions[currentStep]
              ? (robotPositions[currentStep].x === -1 && robotPositions[currentStep].y === -1
                  ? `(${robotPositions[currentStep - 1]?.x}, ${robotPositions[currentStep - 1]?.y}, Facing: ${getFacingDirection(robotPositions[currentStep - 1]?.theta)})` +
                    (robotPositions[currentStep].theta === -1 ? " (Scan Done)" : " (Scanning)")
                  : `(${robotPositions[currentStep].x}, ${robotPositions[currentStep].y}, Facing: ${getFacingDirection(robotPositions[currentStep].theta)})`)
              : "N/A"}
            <br />
            <span className="font-bold">Next Position:</span>{" "}
            {robotPositions[currentStep + 1]
              ? (robotPositions[currentStep + 1].x === -1 && robotPositions[currentStep + 1].y === -1
                  ? `(${robotPositions[currentStep].x}, ${robotPositions[currentStep].y}, Facing: ${getFacingDirection(robotPositions[currentStep].theta)})` +
                    (robotPositions[currentStep + 1].theta === -1 ? " (Scan Done)" : " (Scanning)")
                  : `(${robotPositions[currentStep + 1].x}, ${robotPositions[currentStep + 1].y}, Facing: ${getFacingDirection(robotPositions[currentStep + 1].theta)})`)
              : "End"}
            <br />
            <span className="font-bold">Direction:</span>{" "}
            {robotPositions[currentStep + 1]
              ? (() => {
                  const curr = robotPositions[currentStep];
                  const next = robotPositions[currentStep + 1];
                  if (!curr || !next) return "N/A";
                  if (curr.x === -1 && curr.y === -1) return curr.theta === -1 ? "Scan Done" : "Stationary";
                  if (next.x === -1 && next.y === -1) return next.theta === -1 ? "Scan Done" : "Scanning";
                  const dx = next.x - curr.x;
                  const dy = next.y - curr.y;
                  if (dx === 0 && dy === 0) return "Stationary";
                  const thetaDeg = ((curr.theta ?? 0) * 180) / Math.PI;
                  const thetaNorm = ((thetaDeg % 360) + 360) % 360;
                  const moveAngle = Math.atan2(dy, dx) * 180 / Math.PI;
                  const relativeAngle = ((((moveAngle - thetaNorm) % 360) + 540) % 360) - 180; // [-180, 180]
                  return `${getDirection(curr, next)} (relative angle: ${relativeAngle.toFixed(1)}°)`;
                })()
              : "End"}
          </div>
          {/* Start Animation */}
          <Button
            onClick={() => {
              if (startAnimation) {
                // Stop Animation
                setStartAnimation(false);
              } else {
                // Start Animation
                setIsManualAnimation(false);
                setStartAnimation(true);
                if (currentStep === totalSteps - 1) {
                  setCurrentRobotPosition(robotPositions[0]);
                  setCurrentStep(0);
                }
              }
            }}
          >
            <span>{startAnimation ? "Stop Animation" : "Start Animation"}</span>
            {startAnimation ? <FaPause /> : <FaPlay />}
          </Button>

          {/* Slider */}
          <label
            htmlFor="steps-range"
            className="font-bold text-[14px] flex gap-2 items-center"
          >
            <FaChevronLeft
              className="cursor-pointer"
              onClick={() => {
                if (!startAnimation && currentStep - 1 >= 0) {
                  setIsManualAnimation(true);
                  setCurrentStep((prev) => prev - 1);
                }
              }}
            />
            <span>
              Step: {currentStep + 1} / {totalSteps}
            </span>
            <FaChevronRight
              className="cursor-pointer"
              onClick={() => {
                if (!startAnimation && currentStep + 1 < totalSteps) {
                  setIsManualAnimation(true);
                  setCurrentStep((prev) => prev + 1);
                }
              }}
            />
          </label>
          <input
            id="steps-range"
            type="range"
            min={0}
            max={totalSteps - 1}
            value={currentStep}
            onChange={(e) => {
              setCurrentStep(Number(e.target.value));
              setIsManualAnimation(true);
            }}
            onPointerUp={() => setIsManualAnimation(false)}
            step={1}
            className="w-1/2 h-2 bg-orange-900 rounded-lg appearance-none cursor-pointer"
            disabled={startAnimation === true}
          />
        </div>
      )}

      {/* Navigation Grid */}
      <NavigationGrid
        robotPosition={
          currentRobotPosition ??
          (selectedTestEnum === AlgoTestEnum.Custom
            ? { x: startX, y: startY, theta: startTheta }
            : ROBOT_INITIAL_POSITION)
        }
        obstacles={selectedTest.obstacles}
        canAddObstacle={selectedTestEnum === AlgoTestEnum.Custom}
        setSelectedTest={setSelectedTest}
      />

      {/* Custom Test Settings */}
      {selectedTestEnum === AlgoTestEnum.Custom && (
        <div className="mb-4 flex flex-col items-center gap-2">
          <div>
            <label className="mr-2 font-bold">Start Position:</label>
            <select
              value={`${startX},${startY},${startTheta}`}
              onChange={e => {
                const [x, y, theta] = e.target.value.split(',').map(Number);
                setStartX(x);
                setStartY(y);
                setStartTheta(theta);
              }}
            >
              {START_POSITIONS.map((pos, idx) => (
                <option key={idx} value={`${pos.x},${pos.y},${pos.theta}`}>
                  {pos.label}
                </option>
              ))}
            </select>
          </div>
        </div>
      )}
    </CoreContainter>
  );
};

function getDirection(curr: Position, next: Position): string {
  if (!curr || !next) return "N/A";
  if (curr.x === -1 && curr.y === -1) return "Stationary";
  if (next.x === -1 && next.y === -1) return "Scanning";

  // Convert theta from radians to degrees and normalize
  const thetaDeg = ((curr.theta ?? 0) * 180) / Math.PI;
  const thetaNorm = ((thetaDeg % 360) + 360) % 360;

  // Calculate movement vector
  const dx = next.x - curr.x;
  const dy = next.y - curr.y;

  // Determine facing direction vector
  let fx = 0, fy = 0;
  if (thetaNorm === 0) { fx = 1; fy = 0; }        // East
  else if (thetaNorm === 90) { fx = 0; fy = 1; }  // North
  else if (thetaNorm === 180) { fx = -1; fy = 0; } // West
  else if (thetaNorm === 270) { fx = 0; fy = -1; } // South
  else {
    // For non-cardinal, use unit vector
    const thetaRad = (thetaNorm * Math.PI) / 180;
    fx = Math.round(Math.cos(thetaRad));
    fy = Math.round(Math.sin(thetaRad));
  }

  // Dot product to determine forward/reverse
  const dot = dx * fx + dy * fy;
  const isForward = dot > 0;
  const isReverse = dot < 0;

  // Angle between facing direction and movement vector
  const moveAngle = Math.atan2(dy, dx) * 180 / Math.PI;
  const relativeAngle = ((((moveAngle - thetaNorm) % 360) + 540) % 360) - 180; // [-180, 180]

  // Angle difference for turning (between current and next theta)
  const dtheta = ((next.theta - curr.theta + 2 * Math.PI) % (2 * Math.PI));
  const dthetaDeg = (dtheta * 180) / Math.PI;

  // Use relativeAngle to describe movement direction
  if (dx === 0 && dy === 0) return "Stationary";
  if (isForward) {
    if (Math.abs(relativeAngle) < 10) return "Straight";
    if (relativeAngle > 0) return "Forward Left";
    if (relativeAngle < 0) return "Forward Right";
  }
  if (isReverse) {
    if (Math.abs(relativeAngle) < 10 || Math.abs(relativeAngle) === 180) return "Reverse Straight";
    if (relativeAngle > 0) return "Reverse Left";
    if (relativeAngle < 0) return "Reverse Right";
  }
  if (dtheta !== 0) {
    if (dthetaDeg > 0 && dthetaDeg <= 180) return "Turn Left";
    if (dthetaDeg > 180) return "Turn Right";
  }
  return "Stationary";
}

function getFacingDirection(theta: number | undefined): string {
  if (theta === undefined || theta === null || isNaN(theta)) return "Unknown";
  // Normalize theta to [0, 360)
  const normalized = (theta / Math.PI) * 180;
  if (normalized === 0) return "East";
  if (normalized === 90 || normalized === -270) return "North";
  if (normalized === 180 || normalized === -180) return "West";
  if (normalized === 270 || normalized === -90) return "South";
  // For non-cardinal angles, show closest direction
  //if (normalized > 0 && normalized < 90) return "Northeast";
  //if (normalized > 90 && normalized < 180) return "Northwest";
  //if (normalized > 180 && normalized < 270) return "Southwest";
  //if (normalized > 270 && normalized < 360) return "Southeast";
  return "Unknown";
}
