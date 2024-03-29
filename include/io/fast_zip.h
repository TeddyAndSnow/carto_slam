#pragma once

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <string>

namespace carto_slam
{
    namespace io
    {
        inline void FastGzipString(const std::string &uncompressed,
                                   std::string *compressed)
        {
            boost::iostreams::filtering_ostream out;
            out.push(
                boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));
            out.push(boost::iostreams::back_inserter(*compressed));
            boost::iostreams::write(out,
                                    reinterpret_cast<const char *>(uncompressed.data()),
                                    uncompressed.size());
        }

        inline void FastGunzipString(const std::string &compressed,
                                     std::string *decompressed)
        {
            boost::iostreams::filtering_ostream out;
            out.push(boost::iostreams::gzip_decompressor());
            out.push(boost::iostreams::back_inserter(*decompressed));
            boost::iostreams::write(out, reinterpret_cast<const char *>(compressed.data()),
                                    compressed.size());
        }
    } // namespace io
} // namespace carto_slam