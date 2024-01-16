#include "person_tracker.hpp"
#include <chrono>

PerformanceMetrics metrics;
struct Initialize inputs;
std::vector<std::string> labels;
std::string FLAGS_label = "";
std::unique_ptr<ModelBase> detectionModel;
std::unique_ptr<PedestrianTracker> tracker;
ov::Core core;
ov::InferRequest req;
bool should_keep_tracking_info;


PersonTracker::PersonTracker()
{
    std::cout << "person tracker initialized" << std::endl;
};

PersonTracker::~PersonTracker()
{

};

struct Person PersonTracker::TrackPerson(cv::Mat frame,
// cv::Mat PersonTracker::TrackPerson(cv::Mat frame,
                    std::unique_ptr<ModelBase> &detectionModel,
                    ov::InferRequest req,
                    int32_t person_label,
                    std::unique_ptr<PedestrianTracker> &tracker,
                    double video_fps,
                    unsigned frameIdx) 
{
    struct Person tracked_person;

    detectionModel->preprocess(ImageInputData(frame), req);
    
    req.infer();

    InferenceResult res;

    
    res.internalModelData = std::make_shared<InternalImageModelData>(frame.cols, frame.rows);

    res.metaData = std::make_shared<ImageMetaData>(frame, std::chrono::steady_clock::now());

    for (const auto &outName : detectionModel->getOutputsNames())
    {
        const auto &outTensor = req.get_tensor(outName);

        if (ov::element::i32 == outTensor.get_element_type())
        {
            res.outputsData.emplace(outName, outTensor);
        }
        else
        {
            res.outputsData.emplace(outName, outTensor);
        }
    }

    auto result = (detectionModel->postprocess(res))->asRef<DetectionResult>();

    TrackedObjects detections;

    for (size_t i = 0; i < result.objects.size(); i++)
    {
        TrackedObject object;
        object.confidence = result.objects[i].confidence;

        const float frame_width_ = static_cast<float>(frame.cols);
        const float frame_height_ = static_cast<float>(frame.rows);
        object.frame_idx = result.frameId;

        const float x0 = std::min(std::max(0.0f, result.objects[i].x / frame_width_), 1.0f) * frame_width_;
        const float y0 = std::min(std::max(0.0f, result.objects[i].y / frame_height_), 1.0f) * frame_height_;
        const float x1 =
            std::min(std::max(0.0f, (result.objects[i].x + result.objects[i].width) / frame_width_), 1.0f) *
            frame_width_;
        const float y1 =
            std::min(std::max(0.0f, (result.objects[i].y + result.objects[i].height) / frame_height_), 1.0f) *
            frame_height_;

        object.rect = cv::Rect2f(cv::Point(static_cast<int>(round(static_cast<double>(x0))),
                                           static_cast<int>(round(static_cast<double>(y0)))),
                                 cv::Point(static_cast<int>(round(static_cast<double>(x1))),
                                           static_cast<int>(round(static_cast<double>(y1)))));

        if (object.rect.area() > 0 &&
            (static_cast<int>(result.objects[i].labelID) == person_label || person_label == -1))
        {
            detections.emplace_back(object);
        }
    }

    // timestamp in milliseconds
    uint64_t cur_timestamp = static_cast<uint64_t>(1000.0 / video_fps * frameIdx);
    tracker->Process(frame, detections, cur_timestamp);

    // Drawing colored "worms" (tracks).
    frame = tracker->DrawActiveTracks(frame);

    // Drawing all detected objects on a frame by BLUE COLOR
    for (const auto &detection : detections)
    {
        cv::rectangle(frame, detection.rect, cv::Scalar(255, 0, 0), 3);
    }

    // Drawing tracked detections only by RED color and print ID and detection
    // confidence level.
    std::vector< std::vector<int> > rois;
    std::vector< std::string > labels;

    for (const auto &detection : tracker->TrackedDetections())
    {
        // std::cout << "SANITY CHECK" << std::endl;
        cv::rectangle(frame, detection.rect, cv::Scalar(0, 0, 255), 3);

        std::vector<int> roi{detection.rect.x, detection.rect.y, detection.rect.width, detection.rect.height};
        rois.push_back(roi);

        labels.push_back(std::to_string(detection.object_id));

        std::string text =
            std::to_string(detection.object_id) + " conf: " + std::to_string(detection.confidence);
        putHighlightedText(frame,
                           text,
                           detection.rect.tl() - cv::Point{10, 10},
                           cv::FONT_HERSHEY_COMPLEX,
                           0.65,
                           cv::Scalar(0, 0, 255),
                           2);
    }

    cv::Size graphSize{static_cast<int>(frame.cols / 4), 60};
    Presenter presenter("", 10, graphSize);
    auto startTime = std::chrono::steady_clock::now();

    tracked_person.pt_rois = rois;
    tracked_person.pt_labels = labels;
    tracked_person.pt_frame = frame;

    return tracked_person;
};

std::unique_ptr<PedestrianTracker> PersonTracker::CreatePedestrianTracker(const std::string &reid_model,
                                                           const ov::Core &core,
                                                           const std::string &deviceName,
                                                           bool should_keep_tracking_info)
{
    TrackerParams params;

    if (should_keep_tracking_info)
    {
        params.drop_forgotten_tracks = false;
        params.max_num_objects_in_track = -1;
    }

    std::unique_ptr<PedestrianTracker> tracker(new PedestrianTracker(params));

    // Load reid-model.
    std::shared_ptr<IImageDescriptor> descriptor_fast =
        std::make_shared<ResizedImageDescriptor>(cv::Size(16, 32), cv::InterpolationFlags::INTER_LINEAR);
    std::shared_ptr<IDescriptorDistance> distance_fast = std::make_shared<MatchTemplateDistance>();

    tracker->set_descriptor_fast(descriptor_fast);
    tracker->set_distance_fast(distance_fast);

    if (!reid_model.empty())
    {
        ModelConfigTracker reid_config(reid_model);
        reid_config.max_batch_size = 16; // defaulting to 16
        std::shared_ptr<IImageDescriptor> descriptor_strong =
            std::make_shared<Descriptor>(reid_config, core, deviceName);

        if (descriptor_strong == nullptr)
        {
            throw std::runtime_error("[SAMPLES] internal error - invalid descriptor");
        }
        std::shared_ptr<IDescriptorDistance> distance_strong = std::make_shared<CosDistance>(descriptor_strong->size());

        tracker->set_descriptor_strong(descriptor_strong);
        tracker->set_distance_strong(distance_strong);
    }
    else
    {
        slog::warn << "Reid model "
                   << "was not specified. "
                   << "Only fast reidentification approach will be used." << slog::endl;
    }

    return tracker;
};

// cv::Mat PersonTracker::Run(cv::Mat frame, unsigned frameIdx, std::vector< std::vector<float> > rois, std::vector< std::string > labels)
struct Person PersonTracker::Run(cv::Mat frame, unsigned frameIdx, std::vector< std::vector<int> > rois, std::vector< std::string > labels)
{
   
    struct Person ped;
    try
    {
        const char *hp = std::getenv("HOME");
        const char *mp = std::getenv("PATH_TO_MODULES");
        const char *modelp = std::getenv("VZ_MODEL_DIR");
        std::string home_path(hp);
        std::string modules_path(mp);
        std::string models_path(modelp);
        inputs.det_model = models_path + "/pedestrian_tracker/intel/person-detection-retail-0013/FP32/person-detection-retail-0013.xml";
        inputs.reid_model = models_path + "/pedestrian_tracker/intel/person-reidentification-retail-0277/FP32/person-reidentification-retail-0277.xml";
        inputs.detlog_out = modules_path + "/output.mp4";
        inputs.o = modules_path + "/Human-State-Estimation/output_NEU.mp4"; 

        if (frameIdx == 0){
            
            // PerformanceMetrics metrics;

            // struct Initialize inputs;

            // std::vector<std::string> labels;
            // std::string FLAGS_label = "";
            if (!FLAGS_label.empty())
                labels = DetectionModel::loadLabels(FLAGS_label);

            // std::cout<<"Label size: "<<labels.size()<<std::endl;

            // initialize model
            // std::unique_ptr<ModelBase> detectionModel;
            if (inputs.at == "centernet")
            {
                detectionModel.reset(new ModelCenterNet(inputs.det_model, static_cast<float>(inputs.t), labels, inputs.layout_det));
            }
            else if (inputs.at == "ssd")
            {
                detectionModel.reset(
                    new ModelSSD(inputs.det_model, static_cast<float>(inputs.t), inputs.auto_resize, labels, inputs.layout_det));
            }
            else if (inputs.at == "yolo")
            {
                detectionModel.reset(new ModelYolo(inputs.det_model,
                                                static_cast<float>(inputs.t),
                                                inputs.auto_resize,
                                                inputs.yolo_af,
                                                static_cast<float>(inputs.iou_t),
                                                labels,
                                                {},
                                                {},
                                                inputs.layout_det));
            }
            else
            {
                slog::err << "No model type or invalid model type (-at) provided: " << slog::endl;
                // return -1;
            }

            std::vector<std::string> devices{inputs.detector_mode, inputs.reid_mode};

            // ov::Core core;

            auto model = detectionModel->compileModel(
                ConfigFactory::getUserConfig(inputs.d_det, inputs.nireq, inputs.nstreams, inputs.nthreads),
                core);
            req = model.create_infer_request();

            should_keep_tracking_info = inputs.should_save_det_log || inputs.should_print_out;

            tracker =
                CreatePedestrianTracker(inputs.reid_model, core, inputs.reid_mode, should_keep_tracking_info);

        

            auto startTime = std::chrono::steady_clock::now();

            cv::Size graphSize{static_cast<int>(frame.cols / 4), 60};
            Presenter presenter("", 10, graphSize);
        }
        
        ped = TrackPerson(frame, detectionModel, req, inputs.person_label, tracker, 30, frameIdx);


        if (should_keep_tracking_info)
        {
            DetectionLog log = tracker->GetDetectionLog(true);

            if (inputs.should_save_det_log)
                SaveDetectionLogToTrajFile(inputs.detlog_out, log);
            if (inputs.should_print_out)
                PrintDetectionLog(log);
        }

        
        // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

        return ped;

    }
    catch (const std::exception &error)
    {
        slog::err << error.what() << slog::endl;
        // return 1;
    }
    catch (...)
    {
        slog::err << "Unknown/internal exception happened." << slog::endl;
        // return 1;
    }
};
