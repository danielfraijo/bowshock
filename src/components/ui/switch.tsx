import * as SwitchPrimitive from "@radix-ui/react-switch";
import * as React from "react";
import { cn } from "@/lib/utils";

export const Switch = React.forwardRef<
  React.ComponentRef<typeof SwitchPrimitive.Root>,
  React.ComponentPropsWithoutRef<typeof SwitchPrimitive.Root>
>(({ className, ...props }, ref) => (
  <SwitchPrimitive.Root
    className={cn(
      "hud-switch peer inline-flex h-6 w-11 shrink-0 cursor-pointer items-center rounded-none transition-colors disabled:cursor-not-allowed disabled:opacity-40",
      className,
    )}
    {...props}
    ref={ref}
  >
    <SwitchPrimitive.Thumb className="hud-switch-thumb pointer-events-none block h-4 w-4 translate-x-1 rounded-none transition-transform data-[state=checked]:translate-x-[24px]" />
  </SwitchPrimitive.Root>
));
Switch.displayName = "Switch";
