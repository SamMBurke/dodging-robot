import numpy as np
from scipy.optimize import linear_sum_assignment

from project_pkg.math import angular_lerp, resolve_heading_ambiguity, reconstruct_corners


class KalmanFilter:
    '''
    Standard linear Kalman filter with a constant-velocity motion model for 2D position.
    State: x = [px, py, vx, vy]^T. We only ever measure position (px, py); velocity is inferred.
    '''

    def __init__(self, initial_position, dt, process_noise_std=1.0, measurement_noise_std=0.1):
        self.dt = dt

        # define the state vector of 2D position and 2D velocity
        self.x = np.array([initial_position[0], initial_position[1], 0.0, 0.0])

        # define the state covairance matrix i.e. the uncertainty of the state vector
        # assume that the there is confidence in the initial position, but larger 
        # uncertainty of the inital state velocity
        self.P = np.diag([1.0, 1.0, 10.0, 10.0])

        # define the state transition matrix i.e. how the model changes over time
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # measurement matrix, used to acquire the (x,y) components of the state/covariance matricies
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        self._set_process_noise(process_noise_std)

        # measurement noise covariance matrix
        r = measurement_noise_std**2
        self.R = np.diag([r, r])

    def _set_process_noise(self, process_noise_std):
        # discretized white-noise-acceleration model, decoupled between x and y
        q = process_noise_std**2
        dt2 = self.dt**2
        dt3 = self.dt**3
        dt4 = self.dt**4
        self.Q = q * np.array([
            [dt4 / 4,       0, dt3 / 2,       0],
            [      0, dt4 / 4,       0, dt3 / 2],
            [dt3 / 2,       0,     dt2,       0],
            [      0, dt3 / 2,       0,     dt2]
        ])

    def predict(self, dt=None):
        '''
        Intakes the current system state and outputs the next predicted state and its uncertainty
        '''
        if dt is not None and dt != self.dt:
            self.dt = dt
            self.F[0, 2] = dt
            self.F[1, 3] = dt
            self._set_process_noise(np.sqrt(self.Q[2, 2] / (self.dt ** 2)) if self.dt > 0 else 1.0)

        # predicted state
        self.x = self.F @ self.x

        # predicted estimate covariance
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement):
        '''
        Intakes the current system state and a new measurement and outputs the updated state and its uncertainty
        '''
        z = np.array(measurement)
        y = z - self.H @ self.x                     # innovation / measurement residual
        S = self.H @ self.P @ self.H.T + self.R     # innovation / residual covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)    # Kalman gain

        # update the state estimate and state estimate covariance
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    @property
    def position(self):
        return self.x[:2]

    @property
    def velocity(self):
        return self.x[2:4]
    

class Track:
    '''
    Class object for a single tracked object: a Kalman filter for position/velocity, plus smoothed heading and
    dimensions (handled separately from the KF for the reasons in resolve_heading_ambiguity).

    This class object allows for multiple objects to be tracked individually and 
    implements the update and prediction functions from the KalmanFilter class.
    '''

    def __init__(self, track_id, detected_object, dt):
        self.id = track_id
        self.kf = KalmanFilter(initial_position=detected_object.center, dt=dt)

        self.heading = detected_object.heading
        self.length = detected_object.length
        self.width = detected_object.width
        self.corners = detected_object.corners

        self.age = 1
        self.hits = 1 # tracking total hits
        self.hit_streak = 1 # tracking consecutive hits
        self.time_since_update = 0

    def predict(self, dt):
        self.kf.predict(dt)
        self.age += 1
        if self.time_since_update > 0:
            self.hit_streak = 0
        self.time_since_update += 1
        # keep corners consistent with the predicted center, using last-known heading/size
        self.corners = reconstruct_corners(self.center, self.heading, self.length, self.width)

    def update(self, detected_object, heading_smoothing=0.6, size_smoothing=0.6):
        self.kf.update(detected_object.center)

        heading, length, width = resolve_heading_ambiguity(
            self.heading, detected_object.heading, detected_object.length, detected_object.width
        )

        # get the new heading angle, length, width and corners of the tracked rectangle object
        self.heading = angular_lerp(self.heading, heading, heading_smoothing)
        self.length = (1 - size_smoothing) * self.length + size_smoothing * length
        self.width = (1 - size_smoothing) * self.width + size_smoothing * width
        self.corners = reconstruct_corners(self.center, self.heading, self.length, self.width)

        self.hits += 1
        self.hit_streak += 1
        self.time_since_update = 0

    @property
    def center(self):
        return self.kf.position

    @property
    def velocity(self):
        return self.kf.velocity


class KalmanTracker:
    '''
    This is a multi-object tracker that predicts the tracks forward, associates detected objects to tracks via
    the Hungarian algorithm with a Euclidean-distance cost matrix.

    To ensure that tracking is not done for random objects that barely show up or are unstable, min_hits defines
    how many consecutively matched frames must be found before that track is published. if an object is being tracked
    but then is missed for some reason, then those tracks are removed and is controlled via max_age. To also ensure
    that the tracker is validly tracking a moving object (and to ensure that tracked objects don't get mixed up with
    one another), the max_valid_distance variable ensures that the distance between a predicted track and a detected
    object falls within the range of max_valid_distance (m)
    '''

    def __init__(self, max_age=5, min_hits=3, max_valid_distance=1.5):
        self.max_age = max_age
        self.min_hits = min_hits
        self.max_valid_distance = max_valid_distance
        self.tracks = []
        self._next_id = 0

    def step(self, detected_objects, dt):
        '''
        Advances all tracks by dt, associates detected objects, and returns the confirmed tracks.
        '''
        for track in self.tracks:
            track.predict(dt)

        matches, unmatched_tracks, unmatched_detections = self._associate(detected_objects)

        # update the tracks that have been associated with the newly detected objects
        for track_idx, det_idx in matches:
            self.tracks[track_idx].update(detected_objects[det_idx])

        # for any new, unmatched/associated objects, add them to the list tracks for association and predictions
        for det_idx in unmatched_detections:
            self.tracks.append(Track(self._next_id, detected_objects[det_idx], dt))
            self._next_id += 1

        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]

        # check for valid tracks based on the criteria of max_age before a track is pruned, 
        # consequetive hits via hit_streak, and min hits before a track is considered valid for publishing
        valid_tracks = self._get_confirmed_tracks()

        return valid_tracks

    def _associate(self, detected_objects):
        # if there hasn't been any tracked objects yet, return no matches, and all tracks and objects are unmatched
        if len(self.tracks) == 0 or len(detected_objects) == 0:
            return [], list(range(len(self.tracks))), list(range(len(detected_objects)))

        # create the euclidean distance cost matrix for the hungarian algorithm
        # this matrix is the euclidean distance between the tracked object center and the detected object center
        cost_matrix = np.zeros((len(self.tracks), len(detected_objects)))
        for t, track in enumerate(self.tracks):
            for d, detected_object in enumerate(detected_objects):
                cost_matrix[t, d] = np.linalg.norm(track.center - np.array(detected_object.center))

        # this is the hungarian algorithm via scipy. When you search "hungarian algorithm python" this is what you are recommended
        track_indices, det_indices = linear_sum_assignment(cost_matrix)

        matches = []
        unmatched_tracks = list(range(len(self.tracks)))
        unmatched_detections = list(range(len(detected_objects)))

        # using the max_valid_distance, check the indicies recieved from the Hungarian algorithm
        # to see if the cost matrix at those indicies fall within the valid distance
        for t, d in zip(track_indices, det_indices):
            if cost_matrix[t, d] > self.max_valid_distance:
                continue  # optimal assignment, but too far away to trust
            matches.append((t, d))
            unmatched_tracks.remove(t)
            unmatched_detections.remove(d)

        return matches, unmatched_tracks, unmatched_detections

    def _get_confirmed_tracks(self):
        '''
        Validate the tracks based on the criteria defined in the class description
        '''
        confirmed = []
        for track in self.tracks:
            if track.hit_streak >= self.min_hits and track.time_since_update == 0:
                confirmed.append(track)
        return confirmed

