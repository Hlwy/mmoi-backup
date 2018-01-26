/**
 * @file   IterativeSolver.cpp
 * @brief  
 * @date   Sep 3, 2012
 * @author Yong-Dian Jian
 */

#include <gtsam/linear/IterativeSolver.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <string>

using namespace std;

namespace gtsam {

/*****************************************************************************/
string IterativeOptimizationParameters::getVerbosity() const { return verbosityTranslator(verbosity_); }

/*****************************************************************************/
void IterativeOptimizationParameters::setVerbosity(const string &src) { verbosity_ = verbosityTranslator(src); }

/*****************************************************************************/
void IterativeOptimizationParameters::print() const {
  print(cout);
}

/*****************************************************************************/
void IterativeOptimizationParameters::print(ostream &os) const {
  os << "IterativeOptimizationParameters:" <<  endl
     << "verbosity:     " << verbosityTranslator(verbosity_) <<  endl;
}

/*****************************************************************************/
 ostream& operator<<(ostream &os, const IterativeOptimizationParameters &p) {
  p.print(os);
  return os;
}

/*****************************************************************************/
IterativeOptimizationParameters::Verbosity IterativeOptimizationParameters::verbosityTranslator(const  string &src)  {
   string s = src;  boost::algorithm::to_upper(s);
  if (s == "SILENT") return IterativeOptimizationParameters::SILENT;
  else if (s == "COMPLEXITY") return IterativeOptimizationParameters::COMPLEXITY;
  else if (s == "ERROR") return IterativeOptimizationParameters::ERROR;
  /* default is default */
  else return IterativeOptimizationParameters::SILENT;
}

/*****************************************************************************/
 string IterativeOptimizationParameters::verbosityTranslator(IterativeOptimizationParameters::Verbosity verbosity)  {
  if (verbosity == SILENT) return "SILENT";
  else if (verbosity == COMPLEXITY) return "COMPLEXITY";
  else if (verbosity == ERROR) return "ERROR";
  else return "UNKNOWN";
}

/*****************************************************************************/
VectorValues IterativeSolver::optimize(
    const GaussianFactorGraph &gfg,
    boost::optional<const KeyInfo&> keyInfo,
    boost::optional<const std::map<Key, Vector>&> lambda)
{
  return optimize(
           gfg,
           keyInfo ? *keyInfo : KeyInfo(gfg),
           lambda ? *lambda : std::map<Key,Vector>());
}

/*****************************************************************************/
VectorValues IterativeSolver::optimize (
  const GaussianFactorGraph &gfg,
  const KeyInfo &keyInfo,
  const std::map<Key, Vector> &lambda)
{
  return optimize(gfg, keyInfo, lambda, keyInfo.x0());
}

/****************************************************************************/
KeyInfo::KeyInfo(const GaussianFactorGraph &fg, const Ordering &ordering)
  : ordering_(ordering) {
  initialize(fg);
}

/****************************************************************************/
KeyInfo::KeyInfo(const GaussianFactorGraph &fg)
  : ordering_(Ordering::Natural(fg)) {
  initialize(fg);
}

/****************************************************************************/
void KeyInfo::initialize(const GaussianFactorGraph &fg){
  const map<Key, size_t> colspec = fg.getKeyDimMap();
  const size_t n = ordering_.size();
  size_t start = 0;

  for ( size_t i = 0 ; i < n ; ++i ) {
    const size_t key = ordering_[i];
    const size_t dim = colspec.find(key)->second;
    insert(make_pair(key, KeyInfoEntry(i, dim, start)));
    start += dim ;
  }
  numCols_ = start;
}

/****************************************************************************/
vector<size_t> KeyInfo::colSpec() const {
  std::vector<size_t> result(size(), 0);
  BOOST_FOREACH ( const gtsam::KeyInfo::value_type &item, *this ) {
    result[item.second.index()] = item.second.dim();
  }
  return result;
}

/****************************************************************************/
VectorValues KeyInfo::x0() const {
  VectorValues result;
  BOOST_FOREACH ( const gtsam::KeyInfo::value_type &item, *this ) {
    result.insert(item.first, Vector::Zero(item.second.dim()));
  }
  return result;
}

/****************************************************************************/
Vector KeyInfo::x0vector() const {
  return Vector::Zero(numCols_);
}


}


