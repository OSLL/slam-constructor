#ifndef __SERIALIZATION_H_INCLUDED
#define __SERIALIZATION_H_INCLUDED

#include <memory>
#include <vector>
#include <stdint.h>
#include <iostream>

#ifdef COMPRESSED_SERIALIZATION
#include "roslz4/lz4s.h"
#endif

class Serializer {
public:
    Serializer(const std::vector<char>& data) : data(data) {}
    explicit Serializer(size_t reserved_size = 0) {
        if (reserved_size > 0) data.reserve(reserved_size);
    }

    void reserve(size_t size) {
        data.reserve(size);
    }

    std::vector<char> result() const {
        return data;
    }

    template<typename Type>
    void add_value(Type value) {
        size_t ptr = data.size();
        data.resize(data.size() + sizeof(Type));
        *reinterpret_cast<Type*>(data.data() + ptr) = value;
    }

    void append(const std::vector<char>& new_data) {
        data.reserve(data.size() + new_data.size());
        data.insert(data.end(), new_data.begin(), new_data.end());
    }

#ifdef COMPRESSED_SERIALIZATION
    std::vector<char> compressed() const {
        std::vector<char> res;
        char* output = new char[data.size()];
        unsigned int output_size = data.size();
        if (roslz4_buffToBuffCompress(const_cast<char*>(data.data()), data.size(), output, &output_size, 7) == ROSLZ4_OK) {
            res.assign(output, output + output_size);
        }
        delete[] output;
        return res;
    }
#endif

private:
    std::vector<char> data;
};

template<typename Type>
Serializer& operator<<(Serializer& s, Type value) {
    s.add_value<Type>(value);
    return s;
}

template<>
inline Serializer& operator<<(Serializer& s, const std::vector<char>& data) {
    s.append(data);
    return s;
}

class Deserializer {
public:
    Deserializer(const std::vector<char>& data, size_t start_pos = 0) : ptr(start_pos), data(data) {}

    size_t pos() const {
        return ptr;
    }

    template<typename Type>
    Type read_value() {
        Type value = *reinterpret_cast<const Type*>(data.data() + ptr);
        ptr += sizeof(Type);
        return value;
    }

#ifdef COMPRESSED_SERIALIZATION
    static std::vector<char> decompress(const char* data, size_t data_size, size_t expected_size) {
        std::vector<char> res;
        char *output = new char[expected_size];
        unsigned int output_size = expected_size;
        if (ROSLZ4_OK == roslz4_buffToBuffDecompress(const_cast<char*>(data), data_size, output, &output_size)) {
            res.reserve(output_size);
            res.assign(output, output + output_size);
        }
        delete[] output;
        return res;
    }
#endif

private:
    size_t ptr;
    const std::vector<char>& data;
};

template<typename Type>
Deserializer& operator>>(Deserializer& d, Type& value) {
    value = d.read_value<Type>();
    return d;
}

#endif
