// Copyright 2021 alexander-valov
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Skip uncrustify linting
/* *INDENT-OFF* */

#pragma once

#include <type_traits>

namespace JustInterp {

namespace utils {

    /********************************************************************
     * C++17 compatible implementation of std::experimental::is_detected
     * @see https://en.cppreference.com/w/cpp/experimental/is_detected
     * @see https://people.eecs.berkeley.edu/~brock/blog/detection_idiom.php
     * @see https://blog.tartanllama.xyz/detection-idiom/
     *********************************************************************/
    namespace detail {
        template <template <class...> class Trait, class Enabler, class... Args>
        struct is_detected : std::false_type{};

        template <template <class...> class Trait, class... Args>
        struct is_detected<Trait, std::void_t<Trait<Args...>>, Args...> : std::true_type{};
    }
    template <template <class...> class Trait, class... Args>
    using is_detected = typename detail::is_detected<Trait, void, Args...>::type;


    /********************************************************************
     * Aliases to check support of specific methods or types
     *********************************************************************/
    /* check data() method support */
    template <class T>
    using method_data_t = decltype(std::declval<T>().data());
    template <class T>
    using supports_data = is_detected<method_data_t, T>;

    /* check size() method support */
    template <class T>
    using method_size_t = decltype(std::declval<T>().size());
    template <class T>
    using supports_size = is_detected<method_size_t, T>;

    /* check begin() method support */
    template <class T>
    using method_begin_t = decltype(std::declval<T>().begin());
    template <class T>
    using supports_begin = is_detected<method_begin_t, T>;

    /* check end() method support */
    template <class T>
    using method_end_t = decltype(std::declval<T>().end());
    template <class T>
    using supports_end = is_detected<method_end_t, T>;

    /* check data() is Real* */
    template<class T, class Real>
    using is_real_type_data = std::is_same<Real*, decltype(std::declval<T>().data())>;


    /********************************************************************
     * Alias to detect real-type iterable container
     *********************************************************************/
    template<typename T, typename Real>
    using IsRealContainer = std::enable_if_t<
        std::conjunction_v<
            is_real_type_data<T, Real>,
            supports_begin<T>,
            supports_end<T>,
            supports_data<T>,
            supports_size<T>
        >,
        bool
    >;

    /********************************************************************
     * Alias to detect iterable container of real-type iterable containers
     *********************************************************************/
    template<template<typename> class Container, typename RealContainer, typename Real>
    using IsContainerOfRealContainers = std::enable_if_t<
        std::conjunction_v<
            supports_begin<Container<RealContainer>>,
            supports_end<Container<RealContainer>>,
            supports_data<Container<RealContainer>>,
            supports_size<Container<RealContainer>>,
            is_real_type_data<RealContainer, Real>,
            supports_begin<RealContainer>,
            supports_end<RealContainer>,
            supports_data<RealContainer>,
            supports_size<RealContainer>
        >,
        bool
    >;

    /********************************************************************
     * False type at instantiation time
     * @see https://stackoverflow.com/questions/58694521/what-is-stdfalse-type-or-stdtrue-type
     * @see https://stackoverflow.com/questions/14637356/static-assert-fails-compilation-even-though-template-function-is-called-nowhere
     * @see https://artificial-mind.net/blog/2020/10/03/always-false
     *********************************************************************/
    template <typename T>
    struct always_false : std::false_type {};
}

enum StorageOrder {
    YMajor = 0,
    XMajor = 1
};

enum ExtrapolationType {
    ConstantExtrapolation = 0,
    LinearExtrapolation = 1
};

}
