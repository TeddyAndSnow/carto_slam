
#include "estimator/2d/tsd_value_converter.h"

namespace carto_slam
{
    namespace estimator
    {

        TSDValueConverter::TSDValueConverter(float max_tsd, float max_weight,
                                             ValueConversionTables *conversion_tables)
            : max_tsd_(max_tsd),
              min_tsd_(-max_tsd),
              max_weight_(max_weight),
              tsd_resolution_(32766.f / (max_tsd_ - min_tsd_)),
              weight_resolution_(32766.f / (max_weight_ - min_weight_)),
              value_to_tsd_(
                  conversion_tables->GetConversionTable(min_tsd_, min_tsd_, max_tsd_)),
              value_to_weight_(conversion_tables->GetConversionTable(
                  min_weight_, min_weight_, max_weight)) {}

    } // namespace estimator
} // namespace carto_slam
