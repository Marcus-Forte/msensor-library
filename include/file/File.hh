#pragma once

#include "IFile.hh"
#include <fstream>

class File : public IFile {
public:
  File();
  virtual ~File();
  void open(const std::string &filename) override;
  void write(const char *data, size_t size) override;
  void close() override;
  std::ostream *ostream() override;

private:
  std::ofstream file_;
};