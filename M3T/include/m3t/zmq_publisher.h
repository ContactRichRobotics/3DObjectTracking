/**
 * @file zmq_publisher.h
 * Publisher that use ZMQ to publish data to a subscriber.
 */

#ifndef M3T_INCLUDE_M3T_ZMQ_PUBLISHER_H_
#define M3T_INCLUDE_M3T_ZMQ_PUBLISHER_H_

#include <m3t/common.h>
#include <m3t/link.h>
#include <m3t/publisher.h>
#include <zmq.hpp>

class ZMQPublisher : public m3t::Publisher {
 public:
  // Constructor
  ZMQPublisher(const std::string &name);
  ZMQPublisher(const std::string &name,
               const std::filesystem::path &metafile_path);
  ZMQPublisher(const std::string &name,
               const std::filesystem::path &metafile_path,
               const std::string &address, int port);

  // Setup method
  bool SetUp() override;

  // Main methods
  bool UpdatePublisher(int iteration) override;

  // Load method
  bool LoadMetaData();

  // Set objects
  bool AddLink(const std::shared_ptr<m3t::Link> &link_ptr);

  // Update poses
  void UpdatePoses();

 private:
  // Variables
  std::string address_{};
  int port_{};
  zmq::context_t context_{};
  zmq::socket_t socket_{};

  std::vector<std::shared_ptr<m3t::Link>> link_ptrs_{};
  std::vector<std::array<double, 16>> pose_matrices_{};
};

#endif
