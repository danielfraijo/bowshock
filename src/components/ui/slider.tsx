import * as SliderPrimitive from "@radix-ui/react-slider";
import * as React from "react";
import { cn } from "@/lib/utils";

export const Slider = React.forwardRef<
  React.ComponentRef<typeof SliderPrimitive.Root>,
  React.ComponentPropsWithoutRef<typeof SliderPrimitive.Root>
>(({ className, ...props }, ref) => (
  <SliderPrimitive.Root
    ref={ref}
    className={cn("hud-slider relative flex h-11 w-full touch-none items-center select-none", className)}
    {...props}
  >
    <SliderPrimitive.Track className="hud-slider-track relative h-1.5 w-full grow overflow-hidden">
      <SliderPrimitive.Range className="hud-slider-range absolute h-full" />
    </SliderPrimitive.Track>
    <SliderPrimitive.Thumb className="hud-slider-thumb" />
  </SliderPrimitive.Root>
));
Slider.displayName = "Slider";
