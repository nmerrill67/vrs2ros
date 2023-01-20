#include <iostream>
#include <cassert>
#include <cstdio>
#include <memory>
#include <fstream>

#include <vrs/ErrorCode.h>
#include <vrs/RecordFileReader.h>
#include <vrs/RecordFormatStreamPlayer.h>

//#include <vrs/oss/aria/AudioMetadata.h>
//#include <vrs/oss/aria/BarometerMetadata.h>
//#include <vrs/oss/aria/BluetoothBeaconMetadata.h>
//#include <vrs/oss/aria/GpsMetadata.h>
#include <vrs/oss/aria/ImageSensorMetadata.h>
#include <vrs/oss/aria/MotionSensorMetadata.h>
//#include <vrs/oss/aria/TimeSyncMetadata.h>
//#include <vrs/oss/aria/WifiBeaconMetadata.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>

using namespace std;
using namespace vrs;

bool CALIB_WRITTEN = false;

void printDataLayout(const CurrentRecord& r, DataLayout& datalayout) {
  fmt::print(
      "{:.3f} {} record, {} [{}]\n",
      r.timestamp,
      toString(r.recordType),
      r.streamId.getName(),
      r.streamId.getNumericName());
  datalayout.printLayoutCompact(cout, "  ");
}

class AriaSensorRosbagWriter : public RecordFormatStreamPlayer {
public:
  AriaSensorRosbagWriter(rosbag::Bag *bag_) : bag(bag_) {
    assert(bag != nullptr);
  }

protected:
  rosbag::Bag *bag = nullptr;
  std::string topic = "no_topic";
  std::string frame_id = "no_frame";
};

class AriaImageRosbagWriter : public AriaSensorRosbagWriter {
public:
  AriaImageRosbagWriter(rosbag::Bag *bag, const string& calib_write_path_) 
    : AriaSensorRosbagWriter(bag), calib_write_path(calib_write_path_) {}

  bool onDataLayoutRead(const CurrentRecord& r, size_t blockIndex, DataLayout& dl) override {

    if (r.streamId.getNumericName() == "1201-1") { // SLAM cam #1
      topic = "/cam0/compressed";
      frame_id = "cam0";
    } else if (r.streamId.getNumericName() == "1201-2") { // SLAM cam #2
      topic = "/cam1/compressed";
      frame_id = "cam1";
    } else if (r.streamId.getNumericName() == "214-1") { // RGB camera
      topic = "/cam2_rgb/compressed";
      frame_id = "cam2_rgb";
    } else {
      fmt::print(stderr, "AriaImageRosbagWriter: Unknown stream ID '{}'.\n", 
          r.streamId.getNumericName());
      std::exit(EXIT_FAILURE);
    }

    if (r.recordType == Record::Type::CONFIGURATION) {
      auto& config = getExpectedLayout<aria::ImageSensorConfigRecordMetadata>(dl, blockIndex);
      // Read config record metadata...
      printDataLayout(r, config);

      // Read the calibration data here for all sensors, only once is needed.
      // Write it to json file for reading later
      if (!CALIB_WRITTEN) {
        fmt::print("\n\nFactory calib: {}\n", config.factoryCalibration.get());
        std::ofstream outfile;
        outfile.open(calib_write_path);
        outfile << config.factoryCalibration.get() << std::endl;
        outfile.close();
        fmt::print("\nFactory calib written to file: {}\n", calib_write_path);
      }

    } else if (r.recordType == Record::Type::DATA) {
      auto& data = getExpectedLayout<aria::ImageSensorDataRecordMetadata>(dl, blockIndex);
      // Read data record metadata...
      printDataLayout(r, data);
    }
    
    return true;
  }

  bool onImageRead(const CurrentRecord& r, size_t /*idx*/, const ContentBlock& cb) override {
    // the image data was not read yet: allocate your own buffer & read!
    vector<uint8_t> frameBytes(cb.getBlockSize());
    const auto& imageSpec = cb.image();
    // Synchronously read the image data, which is jpg compressed with Aria
    if (cb.image().getImageFormat() == ImageFormat::JPG && r.reader->read(frameBytes) == 0) {
      /// do your thing with the jpg data...
      fmt::print(
          "{:.3f} {} [{}]: {}, {} bytes.\n",
          r.timestamp,
          r.streamId.getName(),
          r.streamId.getNumericName(),
          imageSpec.asString(),
          imageSpec.getBlockSize());
        
      assert(frame_id != "no_frame");
      assert(topic != "no_topic");

      // Create the image message
      sensor_msgs::CompressedImage msg;
      msg.header.stamp = ros::Time(r.timestamp);
      msg.header.frame_id = frame_id;
      // TODO the rgb camera looks wrong, like RGB has been swapped with BGR.
      std::string base_encoding = frame_id == "cam2_rgb" ? "rgb8" : "mono8";
      msg.format = base_encoding + "; jpeg compressed";
      msg.data = frameBytes;

      // Write to the bag
      bag->write(topic, msg.header.stamp, msg);
    }
    return true; // read next blocks, if any
  }

private:
  string calib_write_path;
};

class AriaMotionSensorRosbagWriter : public AriaSensorRosbagWriter {
public:
  AriaMotionSensorRosbagWriter(rosbag::Bag *bag) : AriaSensorRosbagWriter(bag) {}

  bool onDataLayoutRead(const CurrentRecord& r, size_t blockIndex, DataLayout& dl) override {

    if (r.streamId.getNumericName() == "1202-1") { // IMU #1
      topic = "/imu0";
      frame_id = "imu0";
    } else if (r.streamId.getNumericName() == "1202-2") { // IMU #2
      topic = "/imu1";
      frame_id = "imu1";
    } else {
      fmt::print(stderr, "AriaMotionSensorRosbagWriter: Unknown stream ID '{}'.\n", 
          r.streamId.getNumericName());
      std::exit(EXIT_FAILURE);
    }

    if (r.recordType == Record::Type::CONFIGURATION) {
      auto& config = getExpectedLayout<aria::MotionSensorConfigRecordMetadata>(dl, blockIndex);
      // Read config record metadata...
      printDataLayout(r, config);
    } else if (r.recordType == Record::Type::DATA) {

      // Unlike images, the IMU data is read here as metadata
      auto& data = getExpectedLayout<aria::MotionSensorDataRecordMetadata>(dl, blockIndex);
      
      // Read data record metadata...
      printDataLayout(r, data);
      
      //assert(data.accelValid);
      //assert(data.gyroValid);

      // Create the IMU message
      sensor_msgs::Imu msg;
      msg.header.stamp = ros::Time(r.timestamp);
      msg.header.frame_id = frame_id;
      std::vector<float> tmp;
      assert(data.gyroRadSec.get(tmp));
      msg.angular_velocity.x = tmp.at(0);
      msg.angular_velocity.y = tmp.at(1);
      msg.angular_velocity.z = tmp.at(2);
      
      assert(data.accelMSec2.get(tmp));
      msg.linear_acceleration.x = tmp.at(0);
      msg.linear_acceleration.y = tmp.at(1);
      msg.linear_acceleration.z = tmp.at(2);

      // Write to the bag
      bag->write(topic, msg.header.stamp, msg);

    }
    
    return true;
  }
  
};

/*
class AriaAudioPlayer : public AriaSensorRosbagWriter {
  bool onDataLayoutRead(const CurrentRecord& r, size_t blockIndex, DataLayout& dl) override {
    if (r.recordType == Record::Type::CONFIGURATION) {
      auto& config = getExpectedLayout<aria::AudioConfigRecordMetadata>(dl, blockIndex);
      // Read config record metadata...
      printDataLayout(r, config);
    } else if (r.recordType == Record::Type::DATA) {
      auto& data = getExpectedLayout<aria::AudioDataRecordMetadata>(dl, blockIndex);
      // Read data record metadata...
      printDataLayout(r, data);
    }
    return true;
  }
  bool onAudioRead(const CurrentRecord& r, size_t blockIdx, const ContentBlock& cb) override {
    const AudioContentBlockSpec& audioSpec = cb.audio();
    assert(audioSpec.getSampleFormat() == AudioSampleFormat::S32_LE);
    vector<int32_t> audioData(audioSpec.getSampleCount() * audioSpec.getChannelCount());
    // actually read the audio data
    if (r.reader->read(audioData) == 0) {
      fmt::print(
          "{:.3f} {} [{}]: {} {}x{} samples.\n",
          r.timestamp,
          r.streamId.getName(),
          r.streamId.getNumericName(),
          audioSpec.asString(),
          audioSpec.getSampleCount(),
          audioSpec.getChannelCount());
    }
    return true;
  }
};

class AriaWifiRosbagWriter : public AriaSensorRosbagWriter {
  bool onDataLayoutRead(const CurrentRecord& r, size_t blockIndex, DataLayout& dl) override {
    if (r.recordType == Record::Type::CONFIGURATION) {
      auto& config = getExpectedLayout<aria::WifiBeaconConfigRecordMetadata>(dl, blockIndex);
      // Read config record metadata...
      printDataLayout(r, config);
    } else if (r.recordType == Record::Type::DATA) {
      auto& data = getExpectedLayout<aria::WifiBeaconDataRecordMetadata>(dl, blockIndex);
      // Read data record metadata...
      printDataLayout(r, data);
    }
    return true;
  }
};

class AriaBluetoothRosbagWriter : public AriaSensorRosbagWriter {
  bool onDataLayoutRead(const CurrentRecord& r, size_t blockIndex, DataLayout& dl) override {
    if (r.recordType == Record::Type::CONFIGURATION) {
      auto& config = getExpectedLayout<aria::BluetoothBeaconConfigRecordMetadata>(dl, blockIndex);
      // Read config record metadata...
      printDataLayout(r, config);
    } else if (r.recordType == Record::Type::DATA) {
      auto& data = getExpectedLayout<aria::BluetoothBeaconDataRecordMetadata>(dl, blockIndex);
      // Read data record metadata...
      printDataLayout(r, data);
    }
    return true;
  }
};

class AriaGpsRosbagWriter : public AriaSensorRosbagWriter {
  bool onDataLayoutRead(const CurrentRecord& r, size_t blockIndex, DataLayout& dl) override {
    if (r.recordType == Record::Type::CONFIGURATION) {
      auto& config = getExpectedLayout<aria::GpsConfigRecordMetadata>(dl, blockIndex);
      // Read config record metadata...
      printDataLayout(r, config);
    } else if (r.recordType == Record::Type::DATA) {
      auto& data = getExpectedLayout<aria::GpsDataRecordMetadata>(dl, blockIndex);
      // Read data record metadata...
      printDataLayout(r, data);
    }
    return true;
  }
};

class AriaBarometerRosbagWriter : public AriaSensorRosbagWriter {
  bool onDataLayoutRead(const CurrentRecord& r, size_t blockIndex, DataLayout& dl) override {
    if (r.recordType == Record::Type::CONFIGURATION) {
      auto& config = getExpectedLayout<aria::BarometerConfigRecordMetadata>(dl, blockIndex);
      // Read config record metadata...
      printDataLayout(r, config);
    } else if (r.recordType == Record::Type::DATA) {
      auto& data = getExpectedLayout<aria::BarometerDataRecordMetadata>(dl, blockIndex);
      // Read data record metadata...
      printDataLayout(r, data);
    }
    return true;
  }
};

class AriaTimeSyncRosbagWriter : public AriaSensorRosbagWriter {
  bool onDataLayoutRead(const CurrentRecord& r, size_t blockIndex, DataLayout& dl) override {
    if (r.recordType == Record::Type::CONFIGURATION) {
      auto& config = getExpectedLayout<aria::TimeSyncConfigRecordMetadata>(dl, blockIndex);
      // Read config record metadata...
      printDataLayout(r, config);
    } else if (r.recordType == Record::Type::DATA) {
      auto& data = getExpectedLayout<aria::TimeSyncDataRecordMetadata>(dl, blockIndex);
      // Read data record metadata...
      printDataLayout(r, data);
    }
    return true;
  }
};
*/

int convert_vrs2rosbag(const string& vrs_file, const string& rosbag, 
    const string& calib_json_path) {
  RecordFileReader reader;
  int status = reader.openFile(vrs_file);
  if (status == SUCCESS) {
    // Open the bad file for writing
    rosbag::Bag bag;
    bag.open(rosbag, rosbag::bagmode::Write);

    vector<unique_ptr<StreamPlayer>> streamPlayers;
    // Map the devices referenced in the file to stream player objects
    // Just ignore the device(s) you do not care for
    for (auto id : reader.getStreams()) {
      unique_ptr<StreamPlayer> streamPlayer;
      switch (id.getTypeId()) {
        case RecordableTypeId::SlamCameraData:
        case RecordableTypeId::RgbCameraRecordableClass:
        //case RecordableTypeId::EyeCameraRecordableClass:
          streamPlayer = make_unique<AriaImageRosbagWriter>(&bag, calib_json_path);
          break;
        case RecordableTypeId::SlamImuData:
        //case RecordableTypeId::SlamMagnetometerData:
          streamPlayer = make_unique<AriaMotionSensorRosbagWriter>(&bag);
          break;
        /*
        case RecordableTypeId::WifiBeaconRecordableClass:
          streamPlayer = make_unique<AriaWifiRosbagWriter>();
          break;
        case RecordableTypeId::StereoAudioRecordableClass:
          streamPlayer = make_unique<AriaAudioPlayer>();
          break;
        case RecordableTypeId::BluetoothBeaconRecordableClass:
          streamPlayer = make_unique<AriaBluetoothRosbagWriter>();
          break;
        case RecordableTypeId::GpsRecordableClass:
          streamPlayer = make_unique<AriaGpsRosbagWriter>();
          break;
        case RecordableTypeId::BarometerRecordableClass:
          streamPlayer = make_unique<AriaBarometerRosbagWriter>();
          break;
        case RecordableTypeId::TimeRecordableClass:
          streamPlayer = make_unique<AriaTimeSyncRosbagWriter>();
          break;
        */
        default:
          fmt::print("Ignoring stream: {}, {}.\n", id.getNumericName(), id.getName());
          break;
      }
      if (streamPlayer) {
        reader.setStreamPlayer(id, streamPlayer.get());
        streamPlayers.emplace_back(std::move(streamPlayer));
      }
    }

    int returncode = 0;
    if (streamPlayers.empty()) {
      fmt::print(stderr, "Found no Aria stream in '{}'...\n", vrs_file);
      returncode = 1;
    } else {
      fmt::print("Found {} Aria streams in '{}'. Converting....\n", streamPlayers.size(), vrs_file);
      reader.readAllRecords();
    }
    bag.close();
    return returncode;
  } else {
    fmt::print(stderr, "Failed to open '{}', {}.\n", vrs_file, errorCodeToMessage(status));
    return -1;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vrs2rosbag");
  ros::NodeHandle nh;

  std::string vrs_file = "/media/nate/Elements/aria_pilot/everyday_activities/location_1_indoor/script_1/seq_1/recording_1/recording.vrs";
  nh.param<std::string>("vrs_file", vrs_file, vrs_file); 
  
  std::string rosbag = "/tmp/recording.bag";
  nh.param<std::string>("rosbag", rosbag, rosbag); 
  
  std::string calib_json = "/tmp/calib.json";
  nh.param<std::string>("calib_json", calib_json, calib_json); 

  return convert_vrs2rosbag(vrs_file, rosbag, calib_json);
}
