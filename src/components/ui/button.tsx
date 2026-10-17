import { Slot } from "@radix-ui/react-slot";
import { cva, type VariantProps } from "class-variance-authority";
import * as React from "react";
import { cn } from "@/lib/utils";

const buttonVariants = cva(
  // clip-path / cut-corner hairline / inset focus ring come from the global button rules in styles.css
  "inline-flex items-center justify-center gap-2 whitespace-nowrap rounded-none font-display font-semibold tracking-[0.06em] transition-[background-color,color,box-shadow] duration-150 ease-[cubic-bezier(0.22,1,0.36,1)] disabled:pointer-events-none disabled:opacity-40 [&_svg]:pointer-events-none [&_svg]:size-4 [&_svg]:shrink-0",
  {
    variants: {
      variant: {
        default: "hud-btn-primary bg-accent text-accent-fg hover:bg-[#6ff0ff]",
        secondary: "bg-surface-3 text-fg shadow-[inset_0_0_0_1px_var(--color-line)] hover:shadow-[inset_0_0_0_1px_var(--color-accent)] hover:text-accent",
        outline: "bg-transparent text-fg shadow-[inset_0_0_0_1px_var(--color-line)] hover:bg-accent-dim/50 hover:shadow-[inset_0_0_0_1px_var(--color-accent)] hover:text-accent",
        ghost: "bg-transparent text-muted hover:bg-surface-3 hover:text-fg",
        danger: "bg-warn-dim text-warn shadow-[inset_0_0_0_1px_var(--color-warn)] hover:bg-warn hover:text-bg",
      },
      size: {
        default: "h-11 px-4 text-sm",
        sm: "h-9 px-3 text-xs",
        lg: "h-12 px-5 text-sm",
        icon: "size-11",
      },
    },
    defaultVariants: { variant: "default", size: "default" },
  },
);

export interface ButtonProps
  extends React.ButtonHTMLAttributes<HTMLButtonElement>,
    VariantProps<typeof buttonVariants> {
  asChild?: boolean;
}

export const Button = React.forwardRef<HTMLButtonElement, ButtonProps>(
  ({ className, variant, size, asChild = false, ...props }, ref) => {
    const Comp = asChild ? Slot : "button";
    return <Comp className={cn(buttonVariants({ variant, size, className }))} ref={ref} {...props} />;
  },
);
Button.displayName = "Button";
