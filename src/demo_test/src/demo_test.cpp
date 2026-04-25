#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <vector>

class DemoTestNode : public rclcpp::Node
{
public:
  DemoTestNode() : Node("demo_test"), frame_idx_(0)
  {
    publish_rate_ = this->declare_parameter("publish_rate", 60.0);

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS());

    generateAllFrames();

    auto period_ms = static_cast<int>(1000.0 / publish_rate_);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&DemoTestNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Demo test node started: 4 frames, %.1f Hz, publishing on /image_raw",
                publish_rate_);
  }

private:
  // 在 Bayer 数据上画一个绿色亮点（正确编码到 RGGB 马赛克）
  // 检测算法要求: G/R >= 2.0 且 G/B >= 0.8，所以 R 和 B 必须远小于 G
  static void drawDot(std::vector<uint8_t> &data, int w, int h, int cx, int cy, int radius)
  {
    for (int dy = -radius; dy <= radius; dy++)
    {
      int y = cy + dy;
      if (y < 0 || y >= h)
        continue;
      for (int dx = -radius; dx <= radius; dx++)
      {
        int x = cx + dx;
        if (x < 0 || x >= w)
          continue;
        if (dx * dx + dy * dy > radius * radius)
          continue;

        uint8_t *row = data.data() + y * w;
        if (y % 2 == 0)
          row[x] = (x % 2 == 1) ? 255 : 10; // 偶数行: R(低)=10, G(高)=255
        else
          row[x] = (x % 2 == 0) ? 255 : 10; // 奇数行: G(高)=255, B(低)=10
      }
    }
  }

  void generateAllFrames()
  {
    int w = 1280, h = 1024;

    // --- 帧1: 中心一个大圆点 ---
    {
      std::vector<uint8_t> data(w * h, 10);
      drawDot(data, w, h, 640, 512, 8);
      bayer_frames_.push_back(std::move(data));
      frame_sizes_.emplace_back(w, h);
    }

    // --- 帧2: 上下两个点 ---
    {
      std::vector<uint8_t> data(w * h, 10);
      drawDot(data, w, h, 640, 350, 6);
      drawDot(data, w, h, 640, 700, 6);
      bayer_frames_.push_back(std::move(data));
      frame_sizes_.emplace_back(w, h);
    }

    // --- 帧3: 四个角的小点 ---
    {
      std::vector<uint8_t> data(w * h, 10);
      drawDot(data, w, h, 400, 350, 5);
      drawDot(data, w, h, 880, 350, 5);
      drawDot(data, w, h, 400, 680, 5);
      drawDot(data, w, h, 880, 680, 5);
      bayer_frames_.push_back(std::move(data));
      frame_sizes_.emplace_back(w, h);
    }

    // --- 帧4: 五个点（十字形） ---
    {
      std::vector<uint8_t> data(w * h, 10);
      drawDot(data, w, h, 640, 512, 7); // 中心
      drawDot(data, w, h, 640, 380, 5); // 上
      drawDot(data, w, h, 640, 644, 5); // 下
      drawDot(data, w, h, 510, 512, 5); // 左
      drawDot(data, w, h, 770, 512, 5); // 右
      bayer_frames_.push_back(std::move(data));
      frame_sizes_.emplace_back(w, h);
    }
  }

  void timerCallback()
  {
    if (bayer_frames_.empty())
      return;

    size_t idx = frame_idx_ % bayer_frames_.size();
    auto &data = bayer_frames_[idx];
    auto &size = frame_sizes_[idx];

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_optical_frame";
    msg->encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    msg->height = size.second;
    msg->width = size.first;
    msg->step = size.first;
    msg->data = data;

    publisher_->publish(std::move(msg));
    frame_idx_++;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::vector<uint8_t>> bayer_frames_;
  std::vector<std::pair<int, int>> frame_sizes_;
  size_t frame_idx_;
  double publish_rate_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
