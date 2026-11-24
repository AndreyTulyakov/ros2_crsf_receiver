#include <gtest/gtest.h>
#include "utils.h"

TEST(CrsfMessages, PreservesBothNegativeSnrValues)
{
    LinkStatisticsFrame link{};
    link.uplink_snr = -7;
    link.downlink_snr = -4;
    const auto message = convert_to_link_info(link);
    EXPECT_EQ(message.uplink_snr.data, -7);
    EXPECT_EQ(message.downlink_snr.data, -4);
}
