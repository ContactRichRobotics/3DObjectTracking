#include <m3t/zmq_publisher.h>

ZMQPublisher::ZMQPublisher(const std::string &name) : Publisher(name) {}

ZMQPublisher::ZMQPublisher(const std::string &name,
                           const std::filesystem::path &metafile_path)
    : Publisher(name, metafile_path) {}

ZMQPublisher::ZMQPublisher(const std::string &name,
                           const std::filesystem::path &metafile_path,
                           const std::string &address, int port)
    : Publisher(name, metafile_path), address_(address), port_(port) {}

bool ZMQPublisher::SetUp() {
  if (set_up_) {
    return true;
  }

  // Load meta data
  if (!LoadMetaData()) {
    return false;
  }

  // Create socket
  socket_ = zmq::socket_t(context_, zmq::socket_type::pub);
  socket_.bind("tcp://" + address_ + ":" + std::to_string(port_));

  // Set up publisher
  set_up_ = true;
  return true;
}

bool ZMQPublisher::UpdatePublisher(int iteration) {
  if (!set_up_) {
    return false;
  }

  // Update poses
  UpdatePoses();

  // Serialize the data
  size_t total_size = pose_matrices_.size() * sizeof(std::array<double, 16>);
  std::vector<uint8_t> buffer(total_size);

  uint8_t *ptr = buffer.data();
  for (auto &array : pose_matrices_) {
    memcpy(ptr, array.data(), sizeof(std::array<double, 16>));
    ptr += sizeof(std::array<double, 16>);
  }

  // Send the data
  zmq::message_t message(buffer.begin(), buffer.end());
  socket_.send(message);

  return true;
}

bool ZMQPublisher::LoadMetaData() {
  // Open file storage from yaml
  cv::FileStorage fs;
  if (!m3t::OpenYamlFileStorage(metafile_path_, &fs)) return false;

  // Read parameters from yaml
  if (!m3t::ReadRequiredValueFromYaml(fs, "address", &address_) ||
      !m3t::ReadRequiredValueFromYaml(fs, "port", &port_)) {
    std::cerr << "Could not read all required body parameters from "
              << metafile_path_ << std::endl;
    return false;
  }
  fs.release();
  return true;
}

bool ZMQPublisher::AddLink(const std::shared_ptr<m3t::Link> &link_ptr) {
  link_ptrs_.push_back(link_ptr);
  return true;
}

void ZMQPublisher::UpdatePoses() {
  pose_matrices_.clear();
  for (const auto &link_ptr : link_ptrs_) {
    auto &pose_matrix = link_ptr->link2world_pose().matrix();
    std::array<double, 16> pose_matrix_array{};
    for (int i = 0; i < 16; ++i) {
      pose_matrix_array[i] = pose_matrix.data()[i];
    }
    pose_matrices_.push_back(pose_matrix_array);
  }
}