import os

from os.path import exists, abspath, dirname

# HACK manually edited values...
HERE = dirname(abspath(__file__)).replace("\\", "/")

# DEBUG = True
DEBUG = False

ALICEVISION = '/'.join(["D:", "dev", "Meshroom", "Meshroom-2021.1.0", "aliceVision"])

SCENE_NAME = "angel"

RECONSTRUCTION = "v1_initial"
# RECONSTRUCTION = "v1_shortened"

# SIM_MODE = "cv"
# SIM_MODE = "drone"
SIM_MODE = "drone2"

IMAGES_FOLDER = '/'.join([HERE, SCENE_NAME, RECONSTRUCTION, SIM_MODE, "images"])
OUTPUT_FOLDER = '/'.join([HERE, SCENE_NAME, RECONSTRUCTION, SIM_MODE, "meshroom"])
MESHROOM_CACHE = '/'.join([HERE, SCENE_NAME, RECONSTRUCTION, SIM_MODE, "meshroom", "cache"])

CameraInit = '/'.join([MESHROOM_CACHE, "CameraInit"])
FeatureExtraction = '/'.join([MESHROOM_CACHE, "FeatureExtraction"])
ImageMatching = '/'.join([MESHROOM_CACHE, "ImageMatching"])
FeatureMatching = '/'.join([MESHROOM_CACHE, "FeatureMatching"])
FeatureMatching = '/'.join([MESHROOM_CACHE, "FeatureMatching"])
StructureFromMotion = '/'.join([MESHROOM_CACHE, "StructureFromMotion"])
# ConvertSfMFormat = '/'.join([MESHROOM_CACHE, "ConvertSfMFormat"])

def run_00_cameraInit():
    prog = '/'.join([ALICEVISION, "bin", "aliceVision_cameraInit"])
    opts = [
        "--defaultFieldOfView 45.0 ",
        "--verboseLevel info ",
        '--sensorDatabase "" ',
        "--allowSingleView 1 ",
        '--imageFolder "' + IMAGES_FOLDER + '" ',
        '--output "' + CameraInit + '/cameraInit.sfm" ',
        # '--sensorDatabase "' + '/'.join([ALICEVISION, "share", "aliceVision", "cameraSensors.db"]) + '" ',
        # "--defaultFieldOfView 45.0 ",
        # "--groupCameraFallback folder ",
        # "--allowedCameraModels pinhole,radial1,radial3,brown,fisheye4,fisheye1 ",
        # "--useInternalWhiteBalance True ",
        # "--viewIdMethod metadata ",
        # "--verboseLevel info ",
        # '--output "' + CameraInit + '/cameraInit.sfm" ',
        # "--allowSingleView 1 ",
        # '--input "' + CameraInit + '//viewpoints.sfm"',
    ]
    return prog + "  " + ''.join(opts)


def run_01_featureExtraction():
    prog = '/'.join([ALICEVISION, "bin", "aliceVision_featureExtraction"])
    opts = [
        '--input "' + CameraInit + '/cameraInit.sfm" ',
        "--describerTypes sift ",
        "--describerPreset normal ",
        "--describerQuality normal ",
        "--contrastFiltering GridSort ",
        "--gridFiltering True ",
        "--forceCpuExtraction True ",
        "--maxThreads 0 ",
        "--verboseLevel info ",
        '--output "' + FeatureExtraction + '" ',
        "--rangeStart 0 ",
        "--rangeSize 40",
    ]
    return prog + "  " + ''.join(opts)


def run_02_imageMatching():
    prog = '/'.join([ALICEVISION, "bin", "aliceVision_imageMatching"])
    opts = [
        '--input "' + CameraInit + '/cameraInit.sfm" ',
        '--featuresFolders "' + FeatureExtraction + '" ',
        "--method VocabularyTree ",
        '--tree "' + '/'.join([ALICEVISION, "share", "aliceVision", "vlfeat_K80L3.SIFT.tree"]) + '" ',
        '--weights "" ',
        "--minNbImages 200 ",
        "--maxDescriptors 500 ",
        "--nbMatches 50 ",
        "--verboseLevel info ",
        '--output "' + ImageMatching + '/imageMatches.txt"',
    ]
    return prog + "  " + ''.join(opts)


def run_03_featureMatching():
    prog = '/'.join([ALICEVISION, "bin", "aliceVision_featureMatching"])
    opts = [
        '--input "' + CameraInit + '/cameraInit.sfm" ',
        '--featuresFolders "' + FeatureExtraction + '" ',
        '--imagePairsList "' + ImageMatching + '/imageMatches.txt" ',
        "--describerTypes sift ",
        "--photometricMatchingMethod ANN_L2 ",
        "--geometricEstimator acransac ",
        "--geometricFilterType fundamental_matrix ",
        "--distanceRatio 0.8 ",
        "--maxIteration 2048 ",
        "--geometricError 0.0 ",
        "--knownPosesGeometricErrorMax 5.0 ",
        "--maxMatches 0 ",
        "--savePutativeMatches False ",
        "--crossMatching False ",
        "--guidedMatching False ",
        "--matchFromKnownCameraPoses False ",
        "--exportDebugFiles False ",
        "--verboseLevel info ",
        '--output "' + FeatureMatching + '" ',
        "--rangeStart 0 ",
        "--rangeSize 20",
    ]
    return prog + "  " + ''.join(opts)


def run_04_featureMatching():
    prog = '/'.join([ALICEVISION, "bin", "aliceVision_featureMatching"])
    opts = [
        '--input "' + CameraInit + '/cameraInit.sfm" ',
        '--featuresFolders "' + FeatureExtraction + '" ',
        '--imagePairsList "' + ImageMatching + '/imageMatches.txt" ',
        "--describerTypes sift ",
        "--photometricMatchingMethod ANN_L2 ",
        "--geometricEstimator acransac ",
        "--geometricFilterType fundamental_matrix ",
        "--distanceRatio 0.8 ",
        "--maxIteration 2048 ",
        "--geometricError 0.0 ",
        "--knownPosesGeometricErrorMax 5.0 ",
        "--maxMatches 0 ",
        "--savePutativeMatches False ",
        "--crossMatching False ",
        "--guidedMatching False ",
        "--matchFromKnownCameraPoses False ",
        "--exportDebugFiles False ",
        "--verboseLevel info ",
        '--output "' + FeatureMatching + '" ',
        "--rangeStart 20 ",
        "--rangeSize 20",
    ]
    return prog + "  " + ''.join(opts)


def run_05_incrementalSfM():
    prog = '/'.join([ALICEVISION, "bin", "aliceVision_incrementalSfM"])
    opts = [
        '--input "' + CameraInit + '/cameraInit.sfm" ',
        '--featuresFolders "' + FeatureExtraction + '" ',
        '--matchesFolders "' + FeatureMatching + '" ',
        "--describerTypes sift ",
        "--localizerEstimator acransac ",
        "--observationConstraint Basic ",
        "--localizerEstimatorMaxIterations 4096 ",
        "--localizerEstimatorError 0.0 ",
        "--lockScenePreviouslyReconstructed False ",
        "--useLocalBA True ",
        "--localBAGraphDistance 1 ",
        "--maxNumberOfMatches 0 ",
        "--minNumberOfMatches 0 ",
        "--minInputTrackLength 2 ",
        "--minNumberOfObservationsForTriangulation 2 ",
        "--minAngleForTriangulation 3.0 ",
        "--minAngleForLandmark 2.0 ",
        "--maxReprojectionError 4.0 ",
        "--minAngleInitialPair 5.0 ",
        "--maxAngleInitialPair 40.0 ",
        "--useOnlyMatchesFromInputFolder False ",
        "--useRigConstraint True ",
        "--lockAllIntrinsics False ",
        "--filterTrackForks False ",
        '--initialPairA "" ',
        '--initialPairB "" ',
        "--interFileExtension .abc ",
        "--verboseLevel info ",
        '--output "' + StructureFromMotion + '/sfm.abc" ',
        '--outputViewsAndPoses "' + StructureFromMotion + '/cameras.sfm" ',
        '--extraInfoFolder "' + StructureFromMotion + '/',
    ]
    return prog + "  " + ''.join(opts)


def run_xx_convertSfMFormat():
    prog = '/'.join([ALICEVISION, "bin", "aliceVision_convertSfMFormat"])
    opts = [
        '--input "' + StructureFromMotion + '/sfm.abc" ',
        "--describerTypes sift ",
        "--views True ",
        "--intrinsics True ",
        "--extrinsics True ",
        "--structure True ",
        "--observations True ",
        "--verboseLevel info ",
        # '--output "' + OUTPUT_FOLDER + '/source.ply"',
        # '--output "' + ConvertSfMFormat + '/sfm.ply"',
        '--output "' + StructureFromMotion + '/sfm.ply"',
    ]
    return prog + "  " + ''.join(opts)


def main():
    assert exists(ALICEVISION), ALICEVISION
    assert exists(IMAGES_FOLDER), IMAGES_FOLDER
    assert exists(OUTPUT_FOLDER), OUTPUT_FOLDER

    os.makedirs(MESHROOM_CACHE, exist_ok=True)
    os.makedirs(CameraInit, exist_ok=True)
    os.makedirs(FeatureExtraction, exist_ok=True)
    os.makedirs(ImageMatching, exist_ok=True)
    os.makedirs(FeatureMatching, exist_ok=True)
    os.makedirs(FeatureMatching, exist_ok=True)
    os.makedirs(StructureFromMotion, exist_ok=True)
    # os.makedirs(ConvertSfMFormat, exist_ok=True)

    i = 1
    def run(cmd):
        nonlocal i
        print("\033[92m" + f"({i}/7) > " + cmd + "\033[0m")
        i += 1
        if not DEBUG:
            os.system(cmd)

    cmd_00 = run_00_cameraInit()
    cmd_01 = run_01_featureExtraction()
    cmd_02 = run_02_imageMatching()
    cmd_03 = run_03_featureMatching()
    cmd_04 = run_04_featureMatching()
    cmd_05 = run_05_incrementalSfM()
    cmd_xx = run_xx_convertSfMFormat()

    run(cmd_00)
    run(cmd_01)
    run(cmd_02)
    run(cmd_03)
    run(cmd_04)
    run(cmd_05)
    run(cmd_xx)


if __name__ == "__main__":
    main()
