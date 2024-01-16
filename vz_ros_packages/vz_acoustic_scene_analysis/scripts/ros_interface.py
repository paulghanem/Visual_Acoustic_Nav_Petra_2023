#!/usr/bin/env python3

# Non ROS imports
import pyaudio
import numpy as np
import usb.core
import struct
import sys
import os
from contextlib import contextmanager
import math
from playsound import playsound
from scipy.io.wavfile import write

# ROS specific imports
import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32, String

# Custom imports
path_to_catkin_dir = os.environ.get('CATKIN_DIR')
path_to_mic = str(path_to_catkin_dir) + '/src/third_party_sensors/respeaker_mic'
sys.path.insert(1, str(path_to_mic))
from respeaker_mic.msg import VZ_AudioData
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

path_to_modules = os.environ.get('PATH_TO_MODULES')
path_to_asa = str(path_to_modules) + '/acoustic_scene_analysis'
sys.path.insert(1, str(path_to_asa))
from speaker_verification.USpeakerRecognition import USpeakerRecognition



class ROSInterface:
    def __init__(self):
        # Initialiaze list to store audio segments
        self.wav_list = []
        self.record_count = 0  # Count how many times we've recorded n seconds of audio

        ros_pack = rospkg.RosPack()
        asa_path = ros_pack.get_path('vz_acoustic_scene_analysis')
        wav_path = asa_path + '/wav'
        reg_path = os.environ.get('VZ_REG_DIR')

        # Load audio that is played when the person of interest is identified
        self.audio_poi_detected = wav_path + "/poi_detected.wav"
        self.audio_no_registration_found = wav_path + "/register.wav"

        # Get params
        self.registering = rospy.get_param(
            "/vz_acoustic_scene_analysis/registering")  # get registration flag

        self.target_np_wav_file = str(
            reg_path) + '/asa_reg/audioregistration.csv'

        self.percent_buffer = rospy.get_param(
            "/vz_acoustic_scene_analysis/percent_buffer")
        self.flag = rospy.get_param("/vz_acoustic_scene_analysis/flag")

        self.respeaker_rate = rospy.get_param("/respeaker_mic/respeaker_rate", 16000)

        # Publisher for audio data and speaker verification prediction
        self.audio_data_sub = rospy.Subscriber(
            "/respeaker/wav_data", numpy_msg(VZ_AudioData), self.audio_cb, queue_size=10)
        self.speaker_score_pub = rospy.Publisher(
            "/speaker_score", Float32, queue_size=10)
        self.speaker_pred_pub = rospy.Publisher(
            "/speaker_pred", String, queue_size=10)

        # Setting SpeakerVerification model
        self.spv = USpeakerRecognition.from_hparams(source="speechbrain/spkrec-ecapa-voxceleb",
                                                    savedir="pretrained_models/spkrec-ecapa-voxceleb")


        # for testing audio collection
        file_name = 'audio_out.wav'
        script_path = asa_path + "/scripts/"
        self.dir = os.path.join(script_path, file_name)
        self.ready = False

    def load_registration(self):
        if not self.registering:
            if not os.path.exists(self.target_np_wav_file):
                print("No registration file found!")
                rospy.signal_shutdown("NOT REGISTERING")
                sys.exit(0)
            else:
                target_wave_data = np.loadtxt(self.target_np_wav_file)
                # setting registration data:
                self.spv.set_registration_data(
                    self.respeaker_rate, target_wave_data)
                self.secs = rospy.get_param(
                    "/vz_acoustic_scene_analysis/seconds")
                self.chunk_secs = rospy.get_param(
                    "/vz_acoustic_scene_analysis/chunk_secs")

        if self.registering:
            self.secs = rospy.get_param(
                "/vz_acoustic_scene_analysis/register_seconds")
            self.chunk_secs = rospy.get_param(
                "/vz_acoustic_scene_analysis/register_chunk_secs")
            print("Registering")
            

        secs_to_drop = self.secs/(1/self.percent_buffer)
        self.chunks_to_drop = math.floor(secs_to_drop/self.chunk_secs)

        self.frames = []

        self.respeaker_rate = rospy.get_param("/respeaker_mic/respeaker_rate")
        self.chunk = rospy.get_param("/respeaker_mic/chunk")

        self.arr_length = int(self.respeaker_rate / self.chunk * self.chunk_secs)
        self.ready = True


    def audio_cb(self, msg):
        if hasattr(self, "ready") and self.ready:
            if len(self.frames) < self.arr_length:
                self.frames.append(msg.data)
            else:
                self.store_audio(self.frames)
                self.frames = []
                self.frames.append(msg.data)


    def store_audio(self,recorded_frames):
        self.wav_list.append(recorded_frames)
        self.record_count = len(self.wav_list)
        self.sequence_size = self.secs/self.chunk_secs + 1
        if (self.record_count % self.sequence_size == 0):
            # return_list = True
            print("Collected %d seconds of audio" % (self.secs))
            self.process_audio_loop(self.wav_list)


    def process_audio_loop(self,wav_list):
        flat_list = [
            item for sublist in wav_list for item in sublist]
        flatter_list = [
            item for sublist in flat_list for item in sublist]
        flatter_list_np = np.asarray(flatter_list)

        # remove chunks for buffer
        if not self.registering:
            for chunks in range(self.chunks_to_drop):
                self.wav_list.pop(0)

            # Call of speaker verification
            if (self.flag):
                score, prediction = self.spv.verify_wave_data(
                    self.respeaker_rate, flatter_list_np)
                print("Speaker score = ", score[0].numpy(
                ), " Speaker Prediction =", prediction[0].numpy())
                if prediction[0].numpy():  # If a cough is detected
                    playsound(self.audio_poi_detected)
                self.speaker_score_pub.publish(
                    score[0].numpy())
                score_str = String()
                score_str.data = str(prediction[0].numpy())
                self.speaker_pred_pub.publish(score_str)

        else:
            write(self.dir, 16000, flatter_list_np)
            np.savetxt(self.target_np_wav_file,
                        flatter_list_np, delimiter=",")
            print("saved")
            self.registering = False
            rospy.set_param(
                "/vz_acoustic_scene_analysis/registering", False)
            self.secs = rospy.get_param(
                "/vz_acoustic_scene_analysis/seconds")
            self.chunk_secs = rospy.get_param(
                "/vz_acoustic_scene_analysis/chunk_secs")
            target_wave_data = np.loadtxt(
                self.target_np_wav_file)
            counter = 0
            for chunks in range(len(self.wav_list) - 2):
                self.wav_list.pop(0)
                counter += 1

            # setting registration data:
            self.spv.set_registration_data(
                self.respeaker_rate, target_wave_data)
            secs_to_drop = self.secs/(1/self.percent_buffer)
            self.chunks_to_drop = math.floor(secs_to_drop/self.chunk_secs)
            self.arr_length = int(self.respeaker_rate / self.chunk * self.chunk_secs)


    def cleanup(self):
        print("shutting down")


def audio_capture_main():
    rospy.init_node("audio_capture", disable_signals=True)
    audio = ROSInterface()
    rospy.on_shutdown(audio.cleanup)
    audio.load_registration()
    rospy.spin()


if __name__ == '__main__':
    try:
        audio_capture_main()
    except rospy.ROSInterruptException:
        pass
