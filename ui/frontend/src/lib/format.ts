export function mm(value: number | undefined): string {
  return `${Number(value ?? 0).toFixed(2)} mm`;
}

export function meters(value: number | undefined): string {
  return `${Number(value ?? 0).toFixed(4)} m`;
}

export function rpm(value: number | undefined): string {
  return `${Number(value ?? 0).toFixed(0)} rpm`;
}

export function deg(value: number | undefined): string {
  return `${Number(value ?? 0).toFixed(2)} deg`;
}

export function us(value: number | undefined): string {
  return `${Number(value ?? 0).toFixed(0)} us`;
}

export function percent(value: unknown): string {
  const numeric = typeof value === "number" ? value : Number(value ?? 0);
  return `${Math.max(0, Math.min(100, numeric * 100)).toFixed(0)}%`;
}

export function asBoolText(value: unknown): string {
  return value ? "Si" : "No";
}
