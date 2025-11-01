
#pragma once

#include <cstddef>
#include <iostream>
#include <string>

namespace msensor {

/**
 * @brief Adapter to a file interface.
 *
 */
class IFile {
public:
  virtual void open(const std::string &filename) = 0;
  virtual void write(const char *data, size_t size) = 0;
  virtual void close() = 0;
  virtual std::ostream *ostream() = 0;
};

} // namespace msensor