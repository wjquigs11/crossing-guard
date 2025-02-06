
void logTo::logToAll(String s) {
  if (s.endsWith("\n")) s.remove(s.length() - 1);
  Serial.println(s);
  s = String();
}
