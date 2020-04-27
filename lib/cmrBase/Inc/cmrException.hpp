/*
 * cmrException.hpp
 *
 * Description: This file define the exception handler
 *
 * Author: Feijian.Ni
 * Date: 2020.2.20
 *
 */

#ifndef CMREXCEPTION_HPP_
#define CMREXCEPTION_HPP_

#include "baseStdLibs.hpp"
#include <assert.h>

namespace cmr {

//! error type define
enum cmrErrorType {
  CMR_SUCCESS = 0,
  CMR_FILE_NOT_FOULD,
  CMR_SIZE_NOT_COMPATIBLE,
  CMR_NULL_VALUE,
  CMR_OUT_RANGE,
  CMR_ERROR,

  CMR_ERROR_NUM = CMR_ERROR - CMR_SUCCESS + 1
};

//! error type string descritption
const std::string cmrErrorString[CMR_ERROR_NUM] = {
    "SUCCESS",    "FILE NOT FOULD", "SIZE NOT COMPATIBLE",
    "NULL VALUE", "OUT RANGE",      "ERROR"};
//! exception define
struct cmrException : public std::exception {
  // constructor
  cmrException(std::string msg) { message = msg; }

  // output exception message
  const char *what() throw() { return message.c_str(); }

  std::string message;
};

//! check cmr return error type
inline void _ERRORTYPE_CHECK(cmrErrorType errorType) {
  if (CMR_SUCCESS != errorType) {
    throw cmrException(cmrErrorString[errorType]);
  }
}

//! check if a pointer is null
template <typename T> inline void _NULLPOINTER_CHECK(T ptr, std::string msg) {
  assert(std::is_pointer<T>::value);
  if (!ptr) {
    throw cmrException(msg);
  }
};

} // namespace cmr

#endif