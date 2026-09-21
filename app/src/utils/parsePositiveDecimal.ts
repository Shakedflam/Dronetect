const POSITIVE_DECIMAL = /^(?:\d+(?:\.\d*)?|\.\d+)$/;

/** Parse a positive decimal number from a text field; null when invalid. */
export function parsePositiveDecimal(value: string): number | null {
  if (!POSITIVE_DECIMAL.test(value.trim())) return null;
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed > 0 ? parsed : null;
}
