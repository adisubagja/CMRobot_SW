/*
 * This file give some test example for the filters int
 * lib/filter
 *
 */

#include "ButterworthLPFilter.hpp"
#include "baseStdLibs.hpp"
#include "cmrMatrix.hpp"
#include "movingAverageFilter.hpp"
#include <fstream>
#include <typeinfo>
using namespace cmr;

const std::string g_outputFileName = "./filteredData.txt";

//! read a vector from file stream
bool readVector(cmrVectorXd &vectorData, std::ifstream &infile) {
  for (int i = 0; i < vectorData.size(); i++) {
    if (infile.eof()) {
      return false;
    }
    infile >> vectorData[i];
  }
  return true;
}

//! write a vector to a file stream
void writeVector(const cmrVectorXd &vectorData, std::ofstream &outfile) {
  for (int i = 0; i < vectorData.size(); i++) {
    outfile << vectorData[i] << " ";
  }
  outfile << std::endl;
}

int main(int argc, char *argv[]) {
  for (int i = 0; i < argc; i++) {
    std::cout << argv[i] << std::endl;
  }

  // open input data file and output file
  std::ifstream dataFile;
  dataFile.open(argv[1]);

  // check data dimension
  std::vector<double> firstLine;
  double dataDim = 0;
  double data;
  if (dataFile.is_open()) {
    while ('\n' != dataFile.peek()) {
      dataDim++;
      dataFile >> data;
      firstLine.push_back(data);
    }
  } else {
    std::cout << "can not open input data file: " << argv[1] << std::endl;
    return 1;
  }

  // init filter
  // movingAverageFilter<cmrVectorXd> filter;
  // filter.init(500);

  ButterworthLPFilter<cmrVectorXd> filter(second_order);
  filter.init(5);

  // filt data
  std::ofstream outFile;
  outFile.open(g_outputFileName);
  cmrVectorXd inData = cmrVectorXd::Zero(dataDim);
  cmrVectorXd outData = cmrVectorXd::Zero(dataDim);
  std::cout << "input data dimension is " << dataDim << std::endl;
  for (int i = 0; i < dataDim; i++) {
    inData[i] = firstLine[i];
  }
  outData = filter.runFilter(inData);
  writeVector(outData, outFile);

  while (readVector(inData, dataFile)) {
    outData = filter.runFilter(inData);
    writeVector(outData, outFile);
  }

  dataFile.close();
  outFile.close();
  return 0;
}