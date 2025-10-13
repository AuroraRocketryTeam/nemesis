#pragma once

// Simple response container used across transmitters. Placed in top-level
// include/ so it's available during per-library compilation.
class ResponseStatusContainer
{
  private:
    int code;
    String description;
  public:
    ResponseStatusContainer(int code, String description) : code(code), description(description) {};
    int getCode() const { return code; }
    String getDescription() const { return description; }
};
