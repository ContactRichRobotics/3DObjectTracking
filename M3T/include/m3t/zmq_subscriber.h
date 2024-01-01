// ZMQ Subscriber

#ifndef M3T_INCLUDE_M3T_ZMQ_SUBSCRIBER_H_
#define M3T_INCLUDE_M3T_ZMQ_SUBSCRIBER_H_

#include <m3t/common.h>
#include <m3t/link.h>
#include <m3t/subscriber.h>
#include <zmq.hpp>

namespace m3t {

class ZMQSubscriber : public Subscriber {
 public:
  // Constructors and setup methods
  ZMQSubscriber(const std::string &name);
  ZMQSubscriber(const std::string &name,
                const std::filesystem::path &metafile_path);
  ZMQSubscriber(const std::string &name,
                const std::filesystem::path &metafile_path,
                const std::string &address, int port);
  bool SetUp();

  // Main methods
  bool UpdateSubscriber(int iteration);

  // Set objects
  bool AddLink(const std::shared_ptr<m3t::Link> &link_ptr);

  // Update poses
  void UpdatePoses();

 private:
  // Helper methods
  bool LoadMetaData();

  // Variables
  std::string address_{};
  int port_{};
  zmq::context_t context_{};
  zmq::socket_t socket_{};

  std::vector<std::shared_ptr<m3t::Link>> link_ptrs_{};
  std::vector<std::array<float, 16>> pose_matrices_{};
};

}  // namespace m3t

#endif  // M3T_INCLUDE_M3T_ZMQ_SUBSCRIBER_H_
