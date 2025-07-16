import rclpy
from rclpy.node import Node
from epos4_interfaces.msg import GaitParams, GaitFlag, GaitDone
import random
from random import randrange
import torch
import numpy as np
import os
from scipy.signal import butter, filtfilt
import pickle
from ament_index_python.packages import get_package_share_directory



sdleft_arr = [586, 550, 490,600]
sdright_arr = [608, 570, 510,620]


def predict_step_parameters(step_X, step_y, trial, data_vicon, data_pca, hyper_dict, model_dir, subject="Subject33"):

     device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

     class LSTMModel(torch.nn.Module):
         def __init__(self, input_size, hidden_size, output_size, num_layers, dropout_rate, output_activation):
             super().__init__()
             self.lstm = torch.nn.LSTM(input_size, hidden_size, num_layers,
                                     dropout=dropout_rate if num_layers > 1 else 0.0, batch_first=True)
             self.fc = torch.nn.Linear(hidden_size, output_size)
             self.output_activation = torch.nn.Identity() if output_activation == 'linear' else getattr(torch.nn, output_activation)()

         def forward(self, x):
             out, (hn, _) = self.lstm(x)
             out = self.fc(hn[-1])
             return self.output_activation(out)

     def get_filter_coef(order, cutoff, fs):
         nyquist = 0.5 * fs
         normal_cutoff = cutoff / nyquist
         return butter(order, normal_cutoff, btype='low', analog=False)

     def rename_columns(df, swing):
         mapping = {'1': 'B_', '2': 'LT_', '3': 'RT_'}
         df.columns = [mapping.get(col[-1], "") + col[:-1] if col[-1] in mapping else col for col in df.columns]
         lt_to, rt_to = ("SW_", "ST_") if swing == -1 else ("ST_", "SW_")
         renamed = [lt_to + c[3:] if c.startswith("LT_") else rt_to + c[3:] if c.startswith("RT_") else c for c in df.columns]
         df.columns = renamed
         return df[sorted(df.columns)].to_numpy()

     def filt_array(arr, b, a):
         arr = np.asarray(arr)
         arr_filt = np.empty_like(arr)
         for i in range(arr.shape[1]):
             arr_filt[:, i] = filtfilt(b, a, arr[:, i])
         return arr_filt

     def to_tensor(array):
         return torch.tensor(array, dtype=torch.float32)

     def load_model(model_class, params, X_sample, path):
         model = model_class(
             input_size=X_sample.shape[2],
             hidden_size=params.get('num_lstm_units', 64),
             output_size=1,
             num_layers=params.get('num_lstm_layers', 1),
             dropout_rate=params.get('dropout_rate', 0.0),
             output_activation=params.get('output_activation', 'linear')
         ).to(device)
         model.load_state_dict(torch.load(path, map_location=device))
         model.eval()
         return model

     def lstm_predict(model, X_test):
         X_test = X_test.to(device)
         with torch.no_grad():
             pred_tensor = model(X_test)
         return pred_tensor.cpu().numpy().flatten()[0]

     parameters = ["SD", "SH", "SV"]
     step_duration_min = 34
     filt_b, filt_a = get_filter_coef(4, 6, 100)

     swing_leg = data_vicon['labels'][subject][trial][step_X]['swing_leg']
     subject_norm = data_vicon['subjects_info'][subject]['Height'] * data_vicon['subjects_info'][subject]['Weight'] / 1000
     imu_df = data_vicon['features'][subject][trial][step_X]['imu'].sample(step_duration_min).sort_index()
     imu_processed = rename_columns(imu_df, swing_leg)
     imu_filtered = filt_array(imu_processed, filt_b, filt_a)
     imu_pca = data_pca.transform(imu_filtered).reshape(1, step_duration_min, 6)
     imu_tensor = to_tensor(imu_pca)

     result = {
         "trial": trial,
         "step": step_X,
         "swing_leg": swing_leg,
         "predicted_SD": None,
         "predicted_SH": None,
         "predicted_SV": None,
         "dfratio": None,
         "dbratio": None
    }

     for param in parameters:
         model_path = os.path.join(model_dir, f"LSTM_{param}.pth")
         if not os.path.exists(model_path):
             continue
         model = load_model(LSTMModel, hyper_dict[param], imu_tensor, model_path)
         prediction = lstm_predict(model, imu_tensor)
         result[f"predicted_{param}"] = float(prediction * subject_norm)

     # Compute DF/DB ratios from ground truth
     try:
         gt_SD = data_vicon['labels'][subject][trial][step_y]['SD']
         SDF = data_vicon['labels'][subject][trial][step_X]['SDF']
         SDB = data_vicon['labels'][subject][trial][step_X]['SDB']

         result["dfratio"] = SDF / gt_SD if gt_SD != 0 else 0.0
         result["dbratio"] = SDB / gt_SD if gt_SD != 0 else 0.0
     except KeyError:
         # If any of these labels are missing, leave ratios as None
         pass

     return result



class GaitPrediction(Node):
    def __init__(self):
        super().__init__('gait_prediction')


        pkg_share_dir = get_package_share_directory("epos4_gait_parameter_prediction_node")

        with open(os.path.join(pkg_share_dir, "data", "100_data_vicon.pkl"), "rb") as f:
            self.data_vicon = pickle.load(f)
        with open(os.path.join(pkg_share_dir, "data", "400_lstm_hyper.pkl"), "rb") as f:
            self.hyper_dict = pickle.load(f)
        with open(os.path.join(pkg_share_dir, "data", "700_data_pca.pkl"), "rb") as f:
            self.data_pca = pickle.load(f)

        self.model_dir = os.path.join(pkg_share_dir, "lstm_models")

        # Prepare full list of (trial, step_X, step_Y) tuples
        self.step_sequence = []
        subject = "Subject33"
        trial = 'tro95'
        for trial in self.data_vicon["features"][subject]:
            steps = sorted(self.data_vicon["features"][subject][trial].keys(),
                        key=lambda s: int(s.replace("step", "")))
            
            for i in range(len(steps) - 1):  # (stepX, stepY) pairs
                step_X = steps[i]
                step_Y = steps[i+1]
                swing_leg = self.data_vicon["labels"][subject][trial][step_X]["swing_leg"]
                self.step_sequence.append((trial, step_X, step_Y, swing_leg))

        # Now find index of first step with swing_leg == -1
        for i, (_, _, _, swing_leg) in enumerate(self.step_sequence):
            if swing_leg == -1:
                self.step_index = i
                break
        else:
            self.get_logger().error("No step with swing_leg == -1 found.")
            self.step_index = 0  # fallback

        # Tracking current step
        self.current_trial = None
        self.current_step = None
        self.next_step = None
        self.first_publish = False
        

        # Publishers

        self.param_pub = self.create_publisher(GaitParams, '/gait_params', 10)
        self.flag_pub = self.create_publisher(GaitFlag, '/gait_flag', 10)

        self.create_subscription(GaitDone, '/joint_hip_left/gait_done', self.left_done_cb, 10)
        self.create_subscription(GaitDone, '/joint_hip_right/gait_done', self.right_done_cb, 10)

        random.seed(42) 
        self.index = 0
        self.sdright = float(randrange(450,700))
        self.rsh = float(randrange(100,200))
        self.rsv = float(randrange(50,100))
        self.sdleft = float(randrange(450,700))
        self.lsh = float(randrange(100,200))
        self.lsv = float(randrange(50,100))
        self.dfratio = 0.553 # Random ratio between 1 and 3
        self.dbratio = 0.447
        self.prevflag = 0 # initial gait_flag
        self.swing_leg = 1 # initial swing leg
        self.left_done = True
        self.right_done = True

 

        self.create_timer(0.1, self.check_and_publish)


    def left_done_cb(self, msg: GaitDone):
        if msg.done:
            self.left_done = True
            self.get_logger().info("Left leg completed phase.")

    def right_done_cb(self, msg: GaitDone):
        if msg.done:
            self.right_done = True
            self.get_logger().info("Right leg completed phase.")

    def check_and_publish(self):
        if self.left_done and self.right_done:
            self.publish_predicted_params()
            self.left_done = False
            self.right_done = False

    def publish_predicted_params(self):
        if self.step_index >= len(self.step_sequence):
            self.get_logger().info("All steps completed.")
            return

        trial, step_X, step_Y, swing_leg = self.step_sequence[self.step_index]

        self.swing_leg = swing_leg
        self.current_trial = trial
        self.current_step = step_X
        self.next_step = step_Y

        params = GaitParams()
        publish_flag = GaitFlag()
        params.userheight = self.data_vicon['subjects_info']["Subject33"]["Height"]

       
        result = predict_step_parameters(
                step_X=step_X,
                step_y=step_Y,
                trial=trial,
                data_vicon=self.data_vicon,
                data_pca=self.data_pca,
                hyper_dict=self.hyper_dict,
               model_dir=self.model_dir
            )
        
        self.swing_leg *= -1
        publish_flag.gait_flag = self.prevflag
        self.flag_pub.publish(publish_flag)
            if result["swing_leg"] == -1:
                 params.sdleft = result["predicted_SD"]
                 params.lsh = result["predicted_SH"]
                 params.lsv = result["predicted_SV"]
            elif result["swing_leg"] == 1:
                 params.sdright = result["predicted_SD"]
                 params.rsh = result["predicted_SH"]
                 params.rsv = result["predicted_SV"]

             params.dfratio = result["dfratio"]
             params.dbratio = result["dbratio"]
        self.param_pub.publish(params)

        self.step_index += 1



def main(args=None):
    rclpy.init(args=args)
    node = GaitPrediction()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
