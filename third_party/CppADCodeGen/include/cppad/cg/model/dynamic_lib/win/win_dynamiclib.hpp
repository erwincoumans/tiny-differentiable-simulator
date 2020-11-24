#ifndef CPPAD_CG_WIN_DYNAMICLIB_INCLUDED
#define CPPAD_CG_WIN_DYNAMICLIB_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
 *    Copyright (C) 2018 Joao Leal
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */
#if CPPAD_CG_SYSTEM_WIN

#include <typeinfo>
#include <windows.h>

namespace CppAD {
namespace cg {

/**
 * Useful class to call the compiled source code in a dynamic library.
 * For the Windows Operating System only.
 *
 * @author Joao Leal
 */
template<class Base>
class WindowsDynamicLib : public DynamicLib<Base> {
protected:
    const std::string _dynLibName;
    /// the dynamic library handler
    HINSTANCE _dynLibHandle;
    std::set<WindowsDynamicLibModel<Base>*> _models;
public:

    explicit WindowsDynamicLib(std::string dynLibName) :
        _dynLibName(std::move(dynLibName)),
        _dynLibHandle(nullptr) {

        std::string path;
        if (_dynLibName[0] == '/') {
            path = _dynLibName; // absolute path
        } else if (!(_dynLibName[0] == '.' && _dynLibName[1] == '/') &&
                   !(_dynLibName[0] == '.' && _dynLibName[1] == '.')) {
            path = "./" + _dynLibName; // relative path
        } else {
            path = _dynLibName;
        }

        // load the dynamic library
        _dynLibHandle = LoadLibrary(path.c_str());
        CPPADCG_ASSERT_KNOWN(_dynLibHandle != nullptr, ("Failed to dynamically load library '" + _dynLibName + "': " + dlerror()).c_str())

        // validate the dynamic library
        this->validate();
    }

    inline WindowsDynamicLib(WindowsDynamicLib&& other) noexcept :
            DynamicLib<Base>(std::move(other)),
            _dynLibName(std::move(other._dynLibName)),
            _dynLibHandle(other._dynLibHandle),
            _models(std::move(other._models)) {
        other._dynLibHandle = nullptr;
    }

    WindowsDynamicLib(const WindowsDynamicLib&) = delete;
    WindowsDynamicLib& operator=(const WindowsDynamicLib&) = delete;

    virtual std::unique_ptr<WindowsDynamicLibModel<Base>> modelWindowsDyn(const std::string& modelName) {
        std::unique_ptr<WindowsDynamicLibModel<Base>> m;
        auto it = this->_modelNames.find(modelName);
        if (it == this->_modelNames.end()) {
            return m;
        }
        m.reset(new WindowsDynamicLibModel<Base> (this, modelName));
        _models.insert(m.get());
        return m;
    }

    std::unique_ptr<FunctorGenericModel<Base>> modelFunctor(const std::string& modelName) override final {
        return std::unique_ptr<FunctorGenericModel<Base>>(modelWindowsDyn(modelName).release());
    }

    void* loadFunction(const std::string& functionName, bool required = true) override {
      void* functor = GetProcAddress(_dynLibHandle, functionName.c_str());

        if (required) {
            char *err = dlerror();
            if (err != nullptr)
                throw CGException("Failed to load function '", functionName, "': ", err);
        }

        return functor;
    }

    virtual ~WindowsDynamicLib() {
        for (WindowsDynamicLibModel<Base>* model : _models) {
            model->modelLibraryClosed();
        }

        if (_dynLibHandle != nullptr) {
            if (this->_onClose != nullptr) {
                (*this->_onClose)();
            }

            dlclose(_dynLibHandle);
            _dynLibHandle = nullptr;
        }
    }

protected:

    virtual void destroyed(WindowsDynamicLibModel<Base>* model) {
        _models.erase(model);
    }

    friend class WindowsDynamicLibModel<Base>;

};

} // END cg namespace
} // END CppAD namespace

#endif
#endif
