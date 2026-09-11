import { createFileRoute } from "@tanstack/react-router";
import { Designer } from "@/components/designer";

export const Route = createFileRoute("/")({
  component: Home,
});

function Home() {
  return <Designer />;
}
