import React from "react";
import { TypeAnimation } from "react-type-animation";

export const PageHeader = () => {
  return (
    <div className="flex flex-col items-center justify-center">
      {/* Logo */}
      <img src="/logo.png" alt="logo" />

      {/* Title */}
      <div className="font-rationale font-bold text-[42px] text-center">
        <div>[SC2079]</div>
        <div>Multi-Disciplinary Design Project</div>
      </div>

    </div>
  );
};
