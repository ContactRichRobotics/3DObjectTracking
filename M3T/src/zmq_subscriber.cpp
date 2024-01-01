#include <m3t/zmq_subscriber.h>

namespace m3t {
ZMQSubscriber::ZMQSubscriber(const std::string &name) : Subscriber(name) {}

ZMQSubscriber::ZMQSubscriber(const std::string &name,
                             const std::filesystem::path &metafile_path)
    : Subscriber(name, metafile_path) {}

ZMQSubscriber::ZMQSubscriber(const std::string &name,
                             const std::filesystem::path &metafile_path,
                             const std::string &address, int port)
    : Subscriber(name, metafile_path), address_(address), port_(port) {}

bool ZMQSubscriber::SetUp() {
  if (set_up_) {
    return true;
  }

  // Load meta data
  if (!LoadMetaData()) {
    return false;
  }

  // Create socket
  socket_ = zmq::socket_t(context_, zmq::socket_type::sub);
  socket_.connect("tcp://" + address_ + ":" + std::to_string(port_));

  // Set up subscriber
  socket_.set(zmq::sockopt::subscribe, "");
  set_up_ = true;
  return true;
}

bool ZMQSubscriber::UpdateSubscriber(int iteration) {
  if (!set_up_) {
    return false;
  }

  // Receive the data
  zmq::message_t message;
  auto ret = socket_.recv(message, zmq::recv_flags::dontwait);

  if (ret.has_value()) {
    // Deserialize the data
    size_t total_size = message.size();
    size_t array_size = sizeof(std::array<float, 16>);
    size_t num_poses = total_size / array_size;
    std::vector<uint8_t> buffer(total_size);
    memcpy(buffer.data(), message.data(), total_size);

    pose_matrices_.resize(num_poses);
    uint8_t *ptr = buffer.data();
    for (auto &array : pose_matrices_) {
      memcpy(array.data(), ptr, array_size);
      ptr += array_size;
    }

    // Update poses
    UpdatePoses();
  }
  return true;
}

bool ZMQSubscriber::AddLink(const std::shared_ptr<m3t::Link> &link_ptr) {
  link_ptrs_.push_back(link_ptr);
  return true;
}

void ZMQSubscriber::UpdatePoses() {
  for (size_t i = 0; i < link_ptrs_.size(); ++i) {
    std::cout << link_ptrs_[i]->link2world_pose().matrix() << std::endl;
    Eigen::Matrix4f pose_matrix;
    pose_matrix << pose_matrices_[i][0], pose_matrices_[i][1],
        pose_matrices_[i][2], pose_matrices_[i][3], pose_matrices_[i][4],
        pose_matrices_[i][5], pose_matrices_[i][6], pose_matrices_[i][7],
        pose_matrices_[i][8], pose_matrices_[i][9], pose_matrices_[i][10],
        pose_matrices_[i][11], pose_matrices_[i][12], pose_matrices_[i][13],
        pose_matrices_[i][14], pose_matrices_[i][15];
    m3t::Transform3fA pose(pose_matrix);
    std::cout << pose.matrix() << std::endl;
    if (link_ptrs_[i]->body_ptr() != nullptr) {
      link_ptrs_[i]->body_ptr()->set_body2world_pose(pose);
    }
    else {
      link_ptrs_[i]->set_link2world_pose(pose);
    }
    std::cout << link_ptrs_[i]->link2world_pose().matrix() << std::endl;
  }
}

bool ZMQSubscriber::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  fs.open(metafile_path_.string(), cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return false;
  }

  // Read meta data
  fs["address"] >> address_;
  fs["port"] >> port_;

  return true;
}
//

}  // namespace m3t
