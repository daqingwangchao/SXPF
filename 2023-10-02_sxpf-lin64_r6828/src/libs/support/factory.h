/** Generic factory library (header-only).
 *
 * @file factory.h
 */
#ifndef FACTORY_H_d7c189d6_15d9_42ba_b0a0_7bf9ce12cce5_
#define FACTORY_H_d7c189d6_15d9_42ba_b0a0_7bf9ce12cce5_

#include <functional>
#include <unordered_map>
#include <memory>
#include <typeinfo>

#undef USE_DEMANGLE_
#if defined(__GNUC__)
#define USE_DEMANGLE_
#include <cxxabi.h>
#endif

namespace sxpf {


/** Generic factory template */
template <typename T, typename... GeneratorArgs>
class Factory
{
public:
    using Generator = std::function<T(GeneratorArgs... args)>;

    /// This default constructor ensures that constant initialization is used
    /// for global or static instances of the Factory to enable addition of
    /// generators from global or static registration objects during their
    /// dynamic construction.
    constexpr Factory() noexcept : registry_(nullptr) { }

    /// Add a generator to the factory.
    /// @param name     Key to match when looking up generators in create().
    /// @param maker    Generator object that creates a new instance of a \c T.
    /// @exception std::runtime_error when the name is already present
    void add(const std::string &name, Generator maker)
    {
        check_init_storage();

        auto res = registry_->emplace(name, maker);

        if (res.second)
            return; // T maker successfully inserted

        throw std::runtime_error("Factor already contains an instance named '" +
                                 name + "' of a generator for the type " +
                                 demangle(typeid(T).name()));
    }

    /// Look up a generator and call it to return a newly made instance of \c T.
    /// @param name The name of the selected generator.
    /// @param args Parameter pack (optionally empty) needed to create a \c T.
    /// @return A \c T, newly created using the passed (optional) arguments.
    /// @exception std::runtime_error if the name is not present in the factory
    template <typename... Args>
    T create(const std::string& name, Args&&... args)
    {
        check_init_storage();

        auto it = registry_->find(name);

        if (it == registry_->end())
            throw std::runtime_error(demangle(typeid(T).name()) +
                                     " generator named '" + name +
                                     "' not found");

        return it->second(std::forward<Args>(args)...);
    }

private:
    /// Enable lazy dynamic initialization of the generator registry.
    void check_init_storage()
    {
        if (!registry_)
        {
            registry_ =
                std::make_shared<std::unordered_map<std::string, Generator>>();
        }
    }

    /// Provide human-friendly demangled names of compiler-internal type names.
    /// @param name The compiler-generated type name.
    /// @return The demangled name.
    static std::string demangle(const char *name)
    {
#ifdef USE_DEMANGLE_
        int status = -4;

        char* res = abi::__cxa_demangle(name, nullptr, nullptr, &status);

        const char* const demangled_name = (status == 0) ? res : name;

        std::string ret_val(demangled_name);

        free(res);

        return ret_val;
#else
        return name;
#endif
    }

    /// T generator registry, keyed by name strings.
    std::shared_ptr<std::unordered_map<std::string, Generator>> registry_;
};


} // namespace sxpf

#endif /* FACTORY_H_d7c189d6_15d9_42ba_b0a0_7bf9ce12cce5_ */
