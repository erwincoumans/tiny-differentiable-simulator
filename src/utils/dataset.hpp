#ifndef TINY_DATASET_H
#define TINY_DATASET_H

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace tds {
// Dataset class, for storing multi-axis data.
template <typename Scalar, int Dims>
class Dataset {
 public:
  // Constructs an empty dataset with the given dimensions.
  explicit Dataset() : size_(0) {
    std::fill(shape_.begin(), shape_.end(), 0);
    std::fill(cumprod_shape_.begin(), cumprod_shape_.end(), 0);
  }

  // Constructs a dataset with the given shape.
  template <typename ShapeT>
  explicit Dataset(const ShapeT& shape) {
    Resize(shape);
  }

  // Resizes a dataset. Data is effectively destroyed.
  template <typename ShapeT>
  void Resize(const ShapeT& shape) {
    std::copy(shape.begin(), shape.end(), shape_.begin());
    std::fill(cumprod_shape_.begin(), cumprod_shape_.end(), 1);
    for (int dim = Dims - 2; dim >= 0; --dim) {
      cumprod_shape_[dim] *= shape_[dim + 1] * cumprod_shape_[dim + 1];
    }
    size_ = cumprod_shape_[0] * shape_[0];
    data_.resize(size_);
  }

  // Returns the shape of the dataset.
  const std::array<std::size_t, Dims>& Shape() const { return shape_; }

  // Returns the total size of the dataset.
  std::size_t Size() const { return size_; }

  // Returns a pointer to raw data.
  Scalar* Data() { return data_.data(); }

  // Returns a pointer to raw data.
  const Scalar* Data() const { return data_.data(); }

  // Linear index access, depends on storage order.
  Scalar& operator[](std::size_t linear_index) { return data_[linear_index]; }

  // Linear index access, depends on storage order.
  const Scalar& operator[](std::size_t linear_index) const {
    return data_[linear_index];
  }

  // Multidimensional index access.
  Scalar& operator[](const std::array<std::size_t, Dims>& index) {
    return const_cast<Scalar&>(static_cast<const MyType&>(*this)[index]);
  }

  // Multidimensional index access.
  const Scalar& operator[](const std::array<std::size_t, Dims>& index) const {
    std::size_t linear_index = 0;
    for (int dim = 0; dim < Dims; ++dim) {
      linear_index += cumprod_shape_[dim] * index[dim];
    }
    return data_[linear_index];
  }

 private:
  using MyType = Dataset<Scalar, Dims>;
  std::size_t size_;
  std::array<std::size_t, Dims> shape_;
  std::array<std::size_t, Dims> cumprod_shape_;
  std::vector<Scalar> data_;
};

// Reader for numpy's .npy files.
template <typename Scalar, int Dims>
class NumpyReader {
 public:
  // Opens a file for reading and extracts .npy header. Returns false for
  // invalid files, check ErrorStatus() to find out what happened.
  bool Open(const std::string& path) {
    file_.open(path);
    if (!file_) {
      error_status_ = "Could not open file " + path;
      return false;
    }

    // Read and check magic number.
    char magic_number[7];
    file_.read(magic_number, 6);
    magic_number[6] = 0;
    if (strcmp(magic_number, "\x93NUMPY")) {
      error_status_ =
          std::string("Invalid magic number, got \"") + magic_number + "\"";
      return false;
    }

    // Check version.
    const uint8_t version_major = file_.get();
    const uint8_t version_minor = file_.get();

    // Version 2.0+ have 4byte little endian header length.
    const std::size_t header_len_len = version_major >= 2 ? 4 : 2;
    std::size_t header_len = 0;
    for (int i = 0; i < header_len_len; ++i) {
      header_len += file_.get() << (8 * i);
    }

    // Extract header, using \n termination.
    std::string header;
    std::getline(file_, header);

    // Parse header.
    const std::string descr = ExtractValueString(header, "descr");
    if (descr != "'<f8'") {
      error_status_ =
          "Unsupported dtype " + descr + ". Must be one of ['<f8'].";
      return false;
    }
    const std::size_t element_len = 8;

    // Is this array in fortran order? (first index varies fastest)
    const bool fortran_order =
        !("False" == ExtractValueString(header, "fortran_order"));
    if (fortran_order) {
      error_status_ = "File contains fortran-ordered data.";
      return false;
    }

    // Parse shape information from tuple string.
    shape_ = SplitTuple(ExtractValueString(header, "shape"));
    if (shape_.size() != Dims) {
      error_status_ = "Requested " + std::to_string(Dims) + " but file has " +
                      std::to_string(shape_.size()) +
                      " dims: " + ExtractValueString(header, "shape");
      return false;
    }

    // The total header is padded to the nearest multiple of 64, that's included
    // in header_len.
    data_start_ = 8 + header_len_len + header_len;

    return true;
  }

  // Reads the dataset and returns a Dataset containing the information.
  Dataset<Scalar, Dims> Read() {
    file_.seekg(data_start_);

    Dataset<Scalar, Dims> output(shape_);

    std::vector<char> data;
    data.resize(8 * output.Size());
    file_.read(data.data(), data.size());

    const double* const double_data = reinterpret_cast<double*>(data.data());
    for (std::size_t i = 0; i < output.Size(); ++i) {
      output[i] = double_data[i];
    }

    return output;
  }

  const std::string& ErrorStatus() const { return error_status_; }

 private:
  // Reads a stringified python dict and returns the value for a given key.
  // Not bulletproof.
  std::string ExtractValueString(const std::string& dict,
                                 const std::string& key) {
    // There's a space after the colon.
    const std::size_t begin = dict.find(":", dict.find(key)) + 2;

    int paren_count = 0;
    bool in_squote_str = false;
    bool in_dquote_str = false;
    std::size_t end = begin;
    for (std::size_t i = begin; i < dict.size(); ++i) {
      const bool in_str = in_squote_str && in_dquote_str;
      const bool possibly_terminal = dict[i] == ',' || dict[i] == '}';
      if (!in_str && dict[i] == '(') {
        ++paren_count;
      } else if (!in_str && dict[i] == ')') {
        --paren_count;
      } else if (!in_dquote_str && dict[i] == '\'') {
        in_squote_str = !in_squote_str;
      } else if (!in_squote_str && dict[i] == '"') {
        in_dquote_str = !in_dquote_str;
      } else if (!in_str && paren_count == 0 && possibly_terminal) {
        end = i;
        break;
      }
    }

    return dict.substr(begin, end - begin);
  }

  // Splits a stringified python tuple into a vector. Not bulletproof.
  std::vector<std::size_t> SplitTuple(const std::string& tuple) {
    std::vector<std::size_t> output;
    bool done = false;
    std::size_t last_sep = 0;
    while (!done) {
      const std::size_t next_sep = std::min(tuple.find(",", last_sep + 1),
                                            tuple.find(")", last_sep + 1));
      output.push_back(std::stoi(tuple.substr(last_sep + 1, next_sep)));
      if (tuple[next_sep] == ')') {
        done = true;
      }
      last_sep = next_sep;
    }

    return output;
  }

  std::size_t data_start_ = -1;
  std::vector<std::size_t> shape_;
  std::ifstream file_;
  std::string error_status_;
};
}  // namespace tds

#endif
