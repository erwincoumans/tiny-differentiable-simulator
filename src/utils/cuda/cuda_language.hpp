#pragma once

#include <cppad/cg.hpp>

#include "cuda_variable_name_gen.hpp"

namespace tds {
template <class Base> class LanguageCuda : public CppAD::cg::LanguageC<Base> {
  protected:
  using LanguageC = CppAD::cg::LanguageC<Base>;
    using Node = typename LanguageC::Node;
    using Arg = typename LanguageC::Arg;
    using LanguageC::isSameArgument;
    using LanguageC::_nameGen;
    using LanguageC::getVariableID;
    using LanguageC::auxArrayName_;
    using LanguageC::_streamStack;
    using LanguageC::_indentation;
    using LanguageC::_info;
    using LanguageC::encapsulateIndexPattern;
    using LanguageC::isOffsetBy;
public:
  LanguageCuda(size_t spaces = 2)
      : CppAD::cg::LanguageC<Base>("Float", spaces) {}

  std::unique_ptr<CppAD::cg::LanguageGenerationData<Base>> &getInfo() {
    return this->_info;
  }
  const std::unique_ptr<CppAD::cg::LanguageGenerationData<Base>> &
  getInfo() const {
    return this->_info;
  }

  virtual void pushAtomicForwardOp(Node &atomicFor) {
  using namespace CppAD::cg;
    CPPADCG_ASSERT_KNOWN(
        atomicFor.getInfo().size() == 3,
        "Invalid number of information elements for atomic forward operation")
    int q = atomicFor.getInfo()[1];
    int p = atomicFor.getInfo()[2];
    size_t p1 = p + 1;
    const std::vector<Arg> &opArgs = atomicFor.getArguments();
    CPPADCG_ASSERT_KNOWN(
        opArgs.size() == p1 * 2,
        "Invalid number of arguments for atomic forward operation")

    size_t id = atomicFor.getInfo()[0];
    size_t atomicIndex = this->_info->atomicFunctionId2Index.at(id);

    std::vector<Node *> tx(p1), ty(p1);
    for (size_t k = 0; k < p1; k++) {
      tx[k] = opArgs[0 * p1 + k].getOperation();
      ty[k] = opArgs[1 * p1 + k].getOperation();
    }

    CPPADCG_ASSERT_KNOWN(tx[0]->getOperationType() ==
                             CGOpCode::ArrayCreation,
                         "Invalid array type")
    CPPADCG_ASSERT_KNOWN(p == 0 || tx[1]->getOperationType() ==
                                       CGOpCode::SparseArrayCreation,
                         "Invalid array type")

    CPPADCG_ASSERT_KNOWN(ty[p]->getOperationType() ==
                             CGOpCode::ArrayCreation,
                         "Invalid array type")

    // // tx
    // for (size_t k = 0; k < p1; k++) {
    //   this->printArrayStructInit(this->_ATOMIC_TX, k, tx,
    //                              k); // also does indentation
    // }
    // // ty
    // this->printArrayStructInit(this->_ATOMIC_TY,
    //                            *ty[p]); // also does indentation
    this->_ss.str("");

    assert(p1 == 1); // otherwise see original tx, ty declaration above

    this->_streamStack << "  " << this->_ATOMIC_TX << " = "
                       << this->createVariableName(*tx[0]) << ";\n";
    this->_streamStack << "  " << this->_ATOMIC_TY << " = "
                       << this->createVariableName(*ty[0]) << ";\n";

    const std::string &fun_name = this->_info->atomicFunctionId2Name.at(id);
    this->_streamStack << this->_indentation << fun_name << "_forward_zero("
                       << this->_ATOMIC_TY << ", " << this->_ATOMIC_TX
                       << ");\n";

    // this->_streamStack << this->_indentation <<
    // "atomicFun.forward(atomicFun.libModel, "
    //              << atomicIndex << ", " << q << ", " << p << ", " <<
    //              this->_ATOMIC_TX
    //              << ", &" << this->_ATOMIC_TY << "); // "
    //              << this->_info->atomicFunctionId2Name.at(id) << "\n";

    /**
     * the values of ty are now changed
     */
    this->markArrayChanged(*ty[p]);
  }
  //   virtual void pushAtomicReverseOp(Node &atomicRev) {
  //     CPPADCG_ASSERT_KNOWN(
  //         atomicRev.getInfo().size() == 2,
  //         "Invalid number of information elements for atomic reverse
  //         operation")
  //     int p = atomicRev.getInfo()[1];
  //     size_t p1 = p + 1;
  //     const std::vector<Arg> &opArgs = atomicRev.getArguments();
  //     CPPADCG_ASSERT_KNOWN(
  //         opArgs.size() == p1 * 4,
  //         "Invalid number of arguments for atomic reverse operation")

  //     size_t id = atomicRev.getInfo()[0];
  //     size_t atomicIndex = this->_info->atomicFunctionId2Index.at(id);
  //     std::vector<Node *> tx(p1), px(p1), py(p1);
  //     for (size_t k = 0; k < p1; k++) {
  //       tx[k] = opArgs[0 * p1 + k].getOperation();
  //       px[k] = opArgs[2 * p1 + k].getOperation();
  //       py[k] = opArgs[3 * p1 + k].getOperation();
  //     }

  //     CPPADCG_ASSERT_KNOWN(tx[0]->getOperationType() ==
  //     CGOpCode::ArrayCreation,
  //                          "Invalid array type")
  //     CPPADCG_ASSERT_KNOWN(p == 0 || tx[1]->getOperationType() ==
  //                                        CGOpCode::SparseArrayCreation,
  //                          "Invalid array type")

  //     CPPADCG_ASSERT_KNOWN(px[0]->getOperationType() ==
  //     CGOpCode::ArrayCreation,
  //                          "Invalid array type")

  //     CPPADCG_ASSERT_KNOWN(py[0]->getOperationType() ==
  //                              CGOpCode::SparseArrayCreation,
  //                          "Invalid array type")
  //     CPPADCG_ASSERT_KNOWN(p == 0 || py[1]->getOperationType() ==
  //                                        CGOpCode::ArrayCreation,
  //                          "Invalid array type")

  //     // tx
  //     for (size_t k = 0; k < p1; k++) {
  //       printArrayStructInit(this->_ATOMIC_TX, k, tx, k); // also does
  //       indentation
  //     }
  //     // py
  //     for (size_t k = 0; k < p1; k++) {
  //       printArrayStructInit(this->_ATOMIC_PY, k, py, k); // also does
  //       indentation
  //     }
  //     // px
  //     printArrayStructInit(this->_ATOMIC_PX, *px[0]); // also does
  //     indentation _ss.str("");

  //     this->_streamStack << this->_indentation <<
  //     "atomicFun.reverse(atomicFun.libModel, "
  //                  << atomicIndex << ", " << p << ", " << this->_ATOMIC_TX
  //                  <<
  //                  ", &"
  //                  << this->_ATOMIC_PX << ", " << this->_ATOMIC_PY << ");
  //                  //
  //                  "
  //                  << this->_info->atomicFunctionId2Name.at(id) << "\n";

  //     /**
  //      * the values of px are now changed
  //      */
  //     markArrayChanged(*px[0]);
  //   }

  CppAD::cg::VariableNameGenerator<Base> *getNameGen() {
    return this->_nameGen;
  }

  std::string getAuxArrayName() const { return this->_auxArrayName; }

  void generateArrayContainersDeclaration(std::ostringstream &ss,
                                          int maxForwardOrder = -1,
                                          int maxReverseOrder = -1) override {
    if (maxForwardOrder >= 0 || maxReverseOrder >= 0) {
      ss << this->_spaces << "Float* " << LanguageC::_ATOMIC_TX << ";\n";
      if (maxForwardOrder >= 0)
        ss << this->_spaces << "Float* " << LanguageC::_ATOMIC_TY << ";\n";
      if (maxReverseOrder >= 0) {
        ss << this->_spaces << "Float* " << LanguageC::_ATOMIC_PX << ";\n";
        ss << this->_spaces << "Float* " << LanguageC::_ATOMIC_PY << ";\n";
      }
    }
  }
  
  inline size_t printArrayCreationUsingLoop(
      size_t startPos, Node &array, size_t starti,
      std::vector<const Arg *> &tmpArrayValues)  {
    using namespace CppAD::cg;

    const std::vector<Argument<Base>> &args = array.getArguments();
    const size_t argSize = args.size();
    size_t i = starti + 1;

    std::ostringstream arrayAssign;

    const Argument<Base> &ref = args[starti];
    if (ref.getOperation() != nullptr) {
      //
      const OperationNode<Base> &refOp = *ref.getOperation();
      CGOpCode op = refOp.getOperationType();
      if (op == CGOpCode::Inv) {
        /**
         * from independents array
         */
        for (; i < argSize; i++) {
          if (isSameArgument(args[i], tmpArrayValues[startPos + i]))
            break; // no assignment needed

          if (args[i].getOperation() == nullptr ||
              args[i].getOperation()->getOperationType() != CGOpCode::Inv ||
              !_nameGen->isConsecutiveInIndepArray(
                  *args[i - 1].getOperation(),
                  getVariableID(*args[i - 1].getOperation()),
                  *args[i].getOperation(),
                  getVariableID(*args[i].getOperation()))) {
            break;
          }
        }

        if (i - starti < 3)
          return starti;

        // use loop
        const std::string &indep =
            _nameGen->getIndependentArrayName(refOp, getVariableID(refOp));
        size_t start =
            _nameGen->getIndependentArrayIndex(refOp, getVariableID(refOp));
        long offset = long(start) - starti;
        if (offset == 0)
          arrayAssign << indep << "[i]";
        else
          arrayAssign << indep << "[" << offset << " + i]";

        auto *cudaNameGen =
            static_cast<CudaVariableNameGenerator<Base> *>(this->_nameGen);

        // account for difference between thread-local and global input
        if (starti + offset < cudaNameGen->global_input_dim()) {
          this->_streamStack << this->_indentation << "for(i = " << starti << "; i < "
                       << cudaNameGen->global_input_dim() - offset << "; i++) "
                       << this->_auxArrayName
                       << "[i] = " << cudaNameGen->independent_name() << "["
                       << offset << " + i];\n";
          this->_streamStack << this->_indentation
                       << "for(i = " << cudaNameGen->global_input_dim() - offset
                       << "; i < " << i << "; i++) " << this->_auxArrayName
                       << "[i] = " << cudaNameGen->local_name() << "[" << offset
                       << " + i];\n";
        } else {
          this->_streamStack << this->_indentation << "for(i = " << starti << "; i < " << i
                       << "; i++) " << this->_auxArrayName
                       << "[i] = " << cudaNameGen->local_name() << "[" << offset
                       << " + i];\n";
        }

        return i;

      } else if (op == CGOpCode::LoopIndexedIndep) {
        /**
         * from independents array in a loop
         */
        size_t pos = refOp.getInfo()[1];
        IndexPattern *refIp = this->_info->loopIndependentIndexPatterns[pos];

        LinearIndexPattern *refLIp = nullptr;
        SectionedIndexPattern *refSecp = nullptr;

        if (refIp->getType() == IndexPatternType::Linear) {
          refLIp = dynamic_cast<LinearIndexPattern *>(refIp);
        } else if (refIp->getType() == IndexPatternType::Sectioned) {
          refSecp = dynamic_cast<SectionedIndexPattern *>(refIp);
        } else {
          return starti; // cannot determine consecutive elements
        }

        for (; i < argSize; i++) {
          if (isSameArgument(args[i], tmpArrayValues[startPos + i]))
            break; // no assignment needed

          if (args[i].getOperation() == nullptr ||
              args[i].getOperation()->getOperationType() !=
                  CGOpCode::LoopIndexedIndep) {
            break; // not an independent index pattern
          }

          if (!this->_nameGen->isInSameIndependentArray(
                  refOp, getVariableID(refOp), *args[i].getOperation(),
                  getVariableID(*args[i].getOperation())))
            break;

          pos = args[i].getOperation()->getInfo()[1];
          const IndexPattern *ip = this->_info->loopIndependentIndexPatterns[pos];

          if (!isOffsetBy(ip, refIp, long(i) - long(starti))) {
            break; // different pattern type
          }
        }

        if (i - starti < 3)
          return starti;

        std::unique_ptr<Plane2DIndexPattern> p2dip;
        if (refLIp != nullptr) {
          p2dip.reset(encapsulateIndexPattern(*refLIp, starti));
        } else {
          assert(refSecp != nullptr);
          p2dip.reset(encapsulateIndexPattern(*refSecp, starti));
        }

        std::unique_ptr<OperationNode<Base>> op2(
            OperationNode<Base>::makeTemporaryNode(CGOpCode::LoopIndexedIndep,
                                                   refOp.getInfo(),
                                                   refOp.getArguments()));
        op2->getInfo()[1] =
            (std::numeric_limits<size_t>::max)(); // just to be safe (this would
                                                  // be the index pattern id in
                                                  // the handler)
        op2->getArguments().push_back(_info->auxIterationIndexOp);

        arrayAssign << this->_nameGen->generateIndexedIndependent(*op2, 0, *p2dip);
      } else if (getVariableID(refOp) >= this->_minTemporaryVarID &&
                 op != CGOpCode::LoopIndexedDep &&
                 op != CGOpCode::LoopIndexedTmp && op != CGOpCode::Tmp) {
        /**
         * from temporary variable array
         */
        for (; i < argSize; i++) {
          if (isSameArgument(args[i], tmpArrayValues[startPos + i]))
            break; // no assignment needed
          else if (args[i].getOperation() == nullptr)
            break;

          const OperationNode<Base> &opNode2 = *args[i].getOperation();
          if (getVariableID(opNode2) < this->_minTemporaryVarID)
            break;

          CGOpCode op2 = opNode2.getOperationType();
          if (op2 == CGOpCode::LoopIndexedIndep ||
              op2 == CGOpCode::LoopIndexedDep ||
              op2 == CGOpCode::LoopIndexedTmp || op2 == CGOpCode::Tmp)
            break;

          if (!_nameGen->isConsecutiveInTemporaryVarArray(
                  *args[i - 1].getOperation(),
                  getVariableID(*args[i - 1].getOperation()),
                  *args[i].getOperation(),
                  getVariableID(*args[i].getOperation())))
            break;
        }

        if (i - starti < 3)
          return starti;

        // use loop
        const std::string &tmpName =
            _nameGen->getTemporaryVarArrayName(refOp, getVariableID(refOp));
        size_t start =
            _nameGen->getTemporaryVarArrayIndex(refOp, getVariableID(refOp));
        long offset = long(start) - starti;
        if (offset == 0)
          arrayAssign << tmpName << "[i]";
        else
          arrayAssign << tmpName << "[" << offset << " + i]";

      } else {
        // no loop used
        return starti;
      }

      /**
       * print the loop
       */
      _streamStack << _indentation << "for(i = " << starti << "; i < " << i
                   << "; i++) " << _auxArrayName
                   << "[i] = " << arrayAssign.str() << ";\n";
    } else {
      /**
       * constant value?
       */
      const Base &value = *args[starti].getParameter();
      for (; i < argSize; i++) {
        if (args[i].getParameter() == nullptr ||
            *args[i].getParameter() != value) {
          break; // not the same constant value
        }

        const Argument<Base> *oldArg = tmpArrayValues[startPos + i];
        if (oldArg != nullptr && oldArg->getParameter() != nullptr &&
            *oldArg->getParameter() == value) {
          break; // values are the same (no need to redefine)
        }
      }

      if (i - starti < 3)
        return starti;

      arrayAssign << value;

      /**
       * print the loop
       */
      _streamStack << _indentation << "for(i = " << starti << "; i < " << i
                   << "; i++) " << _auxArrayName
                   << "[i] = " << arrayAssign.str() << ";\n";
    }

    /**
     * update values in the global temporary array
     */
    for (size_t ii = starti; ii < i; ii++) {
      tmpArrayValues[startPos + ii] = &args[ii];
    }

    return i;
  }
};
} // namespace tds