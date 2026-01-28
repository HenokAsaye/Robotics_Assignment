#!/usr/bin/env python3
import sys
import time
import subprocess
import logging

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger('spawn_robot')

def spawn_robot():
    logger.info("Starting robot spawn sequence...")
    logger.info("Waiting for Gazebo world to be ready...")
    
    time.sleep(4)
    
    logger.info("Attempting to spawn robot using gz service...")
    
    try:
        cmd = [
            'gz', 'service', '-s', '/world/addis_ababa_urban/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '20000',
            '--req', "sdf_filename: 'model://medbot/model.sdf'"
        ]
        
        logger.info(f"Running command: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=25)
        
        logger.info(f"Command stdout: {result.stdout}")
        logger.info(f"Command stderr: {result.stderr}")
        logger.info(f"Return code: {result.returncode}")
        
        if result.returncode == 0:
            logger.info("✓ Robot spawned successfully!")
            return 0
        else:
            logger.error(f"✗ Spawn failed with return code {result.returncode}")
            return 1
            
    except subprocess.TimeoutExpired:
        logger.error("✗ Service call timed out - Gazebo world may not be ready")
        return 1
    except Exception as e:
        logger.error(f"✗ Error during spawn: {e}")
        return 1

if __name__ == '__main__':
    sys.exit(spawn_robot())
