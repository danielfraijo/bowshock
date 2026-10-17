// [mesh-refinement] A render error in the viewer / analysis panels degrades to a message instead of a white screen.
import { Component, type ErrorInfo, type ReactNode } from "react";

interface Props {
  label: string;
  children: ReactNode;
  /** Changing this value clears a caught error (e.g. the design parameters), so the next shape gets a fresh try. */
  resetKey?: unknown;
}
interface State {
  error: Error | null;
  key: unknown;
}

export class PanelBoundary extends Component<Props, State> {
  state: State = { error: null, key: this.props.resetKey };
  static getDerivedStateFromError(error: Error): Partial<State> {
    return { error };
  }
  static getDerivedStateFromProps(props: Props, state: State): Partial<State> | null {
    return props.resetKey !== state.key ? { error: null, key: props.resetKey } : null;
  }
  componentDidCatch(error: Error, info: ErrorInfo) {
    console.error(`[${this.props.label}]`, error, info.componentStack);
  }
  render() {
    if (!this.state.error) return this.props.children;
    return (
      <div role="alert" data-testid="panel-error" className="rounded border border-amber-500/40 bg-amber-500/10 p-3 text-xs text-amber-200">
        <div className="font-medium">{this.props.label} could not be drawn for this shape.</div>
        <div className="mt-1 opacity-80">{this.state.error.message || "Unexpected error"}. Change a parameter or switch family to try again.</div>
      </div>
    );
  }
}
