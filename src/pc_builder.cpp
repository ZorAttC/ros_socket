#include "draco/compression/config/compression_shared.h"
#include "draco/compression/encode.h"
#include "draco/compression/expert_encode.h"
#include "draco/core/cycle_timer.h"
#include "draco/io/file_utils.h"
#include "draco/io/mesh_io.h"
#include "draco/io/point_cloud_io.h"
#include "draco/point_cloud/point_cloud_builder.h"
using namespace draco;
int main(int argc, char **argv)
{
    std::vector<float> pos_data_ = {10.f, 0.f, 1.f,
                                    11.f, 1.f, 2.f,
                                    12.f, 2.f, 8.f,
                                    13.f, 4.f, 7.f,
                                    14.f, 5.f, 6.f,
                                    15.f, 6.f, 5.f,
                                    16.f, 1.f, 3.f,
                                    17.f, 1.f, 2.f,
                                    11.f, 1.f, 2.f,
                                    10.f, 0.f, 1.f};
    std::vector<int16_t> intensity_data_ = {100,
                                            200,
                                            500,
                                            700,
                                            400,
                                            400,
                                            400,
                                            100,
                                            100,
                                            100};
    PointCloudBuilder builder;
    builder.Start(10);
    const int pos_att_id =
        builder.AddAttribute(GeometryAttribute::POSITION, 3, DT_FLOAT32);
    const int intensity_att_id =
        builder.AddAttribute(GeometryAttribute::GENERIC, 1, DT_INT16);
    for (PointIndex i(0); i < 10; ++i)
    {
        builder.SetAttributeValueForPoint(pos_att_id, i,
                                          pos_data_.data() + 3 * i.value());
        builder.SetAttributeValueForPoint(intensity_att_id, i,
                                          intensity_data_.data() + i.value());
    }
    std::unique_ptr<PointCloud> pc = builder.Finalize(true);

    const draco::PointAttribute *const att = pc->GetNamedAttribute(draco::GeometryAttribute::POSITION);
    std::vector<std::array<float, 3>> vertex_list;
    std::array<float, 3> value;
    for (draco::AttributeValueIndex i(0); i < static_cast<uint32_t>(att->size()); ++i)
    {
        if (!att->ConvertValue<float, 3>(i, &value[0]))
        {
            return false;
        }
        vertex_list.push_back(value);
        std::cout << "point " << value[0]<<value[1] <<value[2]<< std::endl;
    }

    return 0;
}