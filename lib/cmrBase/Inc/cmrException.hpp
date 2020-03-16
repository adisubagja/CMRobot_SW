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

namespace cmr {

enum cmrErrorType { CMR_SUCCESS = 0, CMR_FILE_NOT_FOULD, CMR_ERROR };

struct cmrException : public std::exception {
  //! constructor
  cmrException(std::string msg) { message = msg; }

  //! output exception message
  const char *what() throw() { return message.c_str(); }

  std::string message;
};

} // namespace cmr

#endif