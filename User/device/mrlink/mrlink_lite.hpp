#pragma once
#include "component/communication/mrlink/mrlink.hpp"
// One implementation only. Specialize MessageTraits/MessageCodec in mr::link.
namespace mr::link::lite {
using namespace mr::link;
template <uint8_t H = 16u> using Instance = mr::link::BasicInstance<H>;
}
#define MRLINK_LITE_TOPIC(T, V) MRLINK_TOPIC(T, V)
#define MRLINK_LITE_MESSAGE(T, V) MRLINK_TOPIC(T, V)
