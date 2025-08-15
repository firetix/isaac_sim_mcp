import asyncio
import os
import time
import json
import requests
import zipfile
import shutil
from pathlib import Path

class Meshy3d:
    def __init__(self):
        """Initialize Meshy3d with API key from environment variables"""
        self.api_key = os.environ.get("MESHY_API_KEY")
        self._working_dir = Path(os.environ.get("USD_WORKING_DIR", "/tmp/usd"))
        self.base_url = "https://api.meshy.ai/v2"
        
        if not self.api_key:
            raise Exception("MESHY_API_KEY environment variable not set, Meshy service is not available until MESHY_API_KEY is set")
        
    
    def _get_headers(self):
        """Get request headers with authorization"""
        return {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
    
    def _download_files_for_completed_task(self, task_id, file_url):
        """
        Process a completed task by downloading and extracting the result file
        
        Args:
            task_id (str): The task ID
            file_url (str): URL to download the result file
            
        Returns:
            str: Path to the extracted task directory
        """
        # Create directories if they don't exist
        tmp_dir = self._working_dir
        tmp_dir.mkdir(parents=True, exist_ok=True)
        
        task_dir = tmp_dir / task_id
        task_dir.mkdir(parents=True, exist_ok=True)
        
        # Download the file
        zip_path = tmp_dir / f"{task_id}.zip"
        response = requests.get(file_url)
        with open(zip_path, "wb") as f:
            f.write(response.content)
        
        # Extract the zip file
        with zipfile.ZipFile(zip_path, 'r') as zip_ref:
            zip_ref.extractall(task_dir)
        
        return str(task_dir)
    
    def monitor_task_status(self, task_id, task_type="text-to-3d"):
        """Monitor task status until succeeded, then download and extract the result file"""
        task_url = f"{self.base_url}/{task_type}/{task_id}"
        elapsed_time_in_seconds = 0
        estimated_time_in_seconds = 180  # Meshy typically takes 2-3 minutes
        while True:
            response = requests.get(task_url, headers=self._get_headers())
            if response.status_code != 200:
                raise Exception(f"Error fetching task status: {response.text}")
                
            task_data = response.json()
            status = task_data.get("status")
            
            if status == "SUCCEEDED":
                # Meshy returns model_urls with different formats
                model_urls = task_data.get("model_urls", {})
                # Try to get USD format first, fallback to GLB
                file_url = model_urls.get("usd") or model_urls.get("glb")
                if not file_url:
                    raise Exception("No suitable model file URL found in the response")
                
                return self._download_files_for_completed_task(task_id, file_url)
            
            if status == "FAILED":
                raise Exception(f"Task failed: {task_data}")
            elif status == "IN_PROGRESS":
                completion_ratio = min(100, int((elapsed_time_in_seconds / estimated_time_in_seconds) * 100))
                print(f"Task {task_id} is generating. Progress: {completion_ratio}% complete. Waiting for completion...")
            
            # Sleep for 10 seconds before checking again (Meshy recommends less frequent polling)
            time.sleep(10)
            elapsed_time_in_seconds += 10
    
    async def monitor_task_status_async(self, task_id, task_type="text-to-3d", on_complete_callback=None):
        """
        Asynchronously monitor task status until succeeded, then download and extract the result file
        
        Args:
            task_id (str): The task ID to monitor
            task_type (str): Type of task ("text-to-3d" or "image-to-3d")
            on_complete_callback (callable, optional): Callback function to call when task completes
                                             with the task directory as argument
        
        Returns:
            str: Path to the extracted task directory
        """
        import asyncio
        
        task_url = f"{self.base_url}/{task_type}/{task_id}"
        elapsed_time_in_seconds = 0
        estimated_time_in_seconds = 180  # Meshy typically takes 2-3 minutes
        
        while True:
            response = requests.get(task_url, headers=self._get_headers())
            if response.status_code != 200:
                raise Exception(f"Error fetching task status: {response.text}")
                
            task_data = response.json()
            status = task_data.get("status")
            
            if status == "SUCCEEDED":
                model_urls = task_data.get("model_urls", {})
                file_url = model_urls.get("usd") or model_urls.get("glb")
                if not file_url:
                    raise Exception("No suitable model file URL found in the response")
                
                result_path = self._download_files_for_completed_task(task_id, file_url)
                
                # Call the callback if provided
                if on_complete_callback and callable(on_complete_callback):
                    on_complete_callback(task_id, status, result_path)
                
                return result_path
            
            if status == "FAILED":
                raise Exception(f"Task failed: {task_data}")
            elif status == "IN_PROGRESS":
                completion_ratio = min(100, int((elapsed_time_in_seconds / estimated_time_in_seconds) * 100))
                print(f"Task {task_id} is generating. Progress: {completion_ratio}% complete. Waiting for completion...")
            
            # Asynchronously sleep for 10 seconds before checking again
            await asyncio.sleep(10)
            elapsed_time_in_seconds += 10
    
    def generate_3d_from_text(self, text_prompt, mode="preview", art_style="realistic", negative_prompt=""):
        """Generate a 3D model from text input and return the task ID"""
        
        payload = {
            "mode": mode,  # "preview" for fast generation, "refine" for high quality
            "prompt": text_prompt,
            "art_style": art_style,  # "realistic", "cartoon", "sculpture", etc.
            "negative_prompt": negative_prompt,
            "seed": 0
        }
        
        response = requests.post(
            f"{self.base_url}/text-to-3d",
            headers=self._get_headers(),
            json=payload
        )
        
        if response.status_code not in [200, 201]:
            raise Exception(f"Error generating 3D model: {response.text}")
        
        return response.json().get("result")
    
    def generate_3d_from_image(self, image_url, mode="preview", enable_pbr=True):
        """Generate a 3D model from an image URL and return the task ID
        
        Args:
            image_url (str): URL of the image to generate 3D model from
            mode (str): Generation mode - "preview" or "refine"
            enable_pbr (bool): Whether to enable PBR materials
            
        Returns:
            str: Task ID for the generation job
        """
        payload = {
            "mode": mode,
            "image_url": image_url,
            "enable_pbr": enable_pbr
        }
        
        response = requests.post(
            f"{self.base_url}/image-to-3d",
            headers=self._get_headers(),
            json=payload
        )
        
        if response.status_code not in [200, 201]:
            raise Exception(f"Error generating 3D model from image: {response.text}")
        
        return response.json().get("result")


def main():
    """Main function to test the Meshy3d class"""
    try:
        # Initialize the Meshy3d class
        meshy = Meshy3d()
        
        # Generate a 3D model from text
        text_prompt = "A gothic castle with 4 towers surrounding a central tower, inspired by Notre-Dame de Paris"
        task_id = meshy.generate_3d_from_text(text_prompt)
        print(f"3D model generation task started with ID: {task_id}")
        
        # Monitor the task and download the result
        result_path = meshy.monitor_task_status(task_id, "text-to-3d")
        print(f"3D model downloaded to: {result_path}")
        
        # Generate a 3D model from an image
        image_url = "https://example.com/sample-image.jpg"
        task_id = meshy.generate_3d_from_image(image_url)
        print(f"3D model generation from image task started with ID: {task_id}")
        
        # Monitor the task and download the result
        result_path = meshy.monitor_task_status(task_id, "image-to-3d")
        print(f"3D model from image downloaded to: {result_path}")
        
    except Exception as e:
        print(f"Error: {str(e)}")

async def test_async():
    # Test initialization
        meshy = Meshy3d()
        assert meshy.api_key, "API key should be set"

        # Test async task monitoring with callback
        def call_back_fn(task_id, status=None, result_path=None):
            print(f"Callback invoked: Task {task_id} status is {status}")
            if result_path:
                print(f"Callback received result path: {result_path}")
            return True
        
        # Test async monitoring
        async_task_id = meshy.generate_3d_from_text("A simple chair")
        assert async_task_id, "Async task ID should be returned"
        print(f"Starting async monitoring for task ID: {async_task_id}")
        
        # Monitor the task asynchronously with callback
        result_path = await meshy.monitor_task_status_async(async_task_id, "text-to-3d", on_complete_callback=call_back_fn)
        print(f"Async monitoring initiated for task ID: {async_task_id}")
        print(f"Async monitoring completed, result path: {result_path}")
        print("Async test completed")
        return result_path

def test():
    """Unit test for the Meshy3d class"""
    try:
        # Test initialization
        meshy = Meshy3d()
        assert meshy.api_key, "API key should be set"

        # Test text generation
        text_prompt = "A fresh apple in red"
        task_id = meshy.generate_3d_from_text(text_prompt)
        assert task_id, "Task ID should be returned"
        print(f"Text generation test passed, task ID: {task_id}")
        result_path = meshy.monitor_task_status(task_id, "text-to-3d")
        result_path_obj = Path(result_path)
        assert result_path_obj.exists(), f"Downloaded file does not exist at {result_path}"
        assert result_path_obj.is_dir(), f"Expected directory at {result_path}"
        
        # Check if there are model files in the extracted directory
        model_files = list(result_path_obj.glob("*.usd")) + list(result_path_obj.glob("*.usda")) + list(result_path_obj.glob("*.usdc")) + list(result_path_obj.glob("*.glb"))
        assert len(model_files) > 0, f"No model files found in {result_path}"
        print(f"Verified: Model file successfully downloaded and extracted to {result_path}")
        
        # Test text generation with Chinese text
        chinese_text_prompt = "一个哥特式的城堡,参考巴黎圣母院"
        task_id = meshy.generate_3d_from_text(chinese_text_prompt)
        assert task_id, "Task ID should be returned"
        print(f"Chinese text generation test passed, task ID: {task_id}")
        result_path = meshy.monitor_task_status(task_id, "text-to-3d")
        result_path_obj = Path(result_path)
        assert result_path_obj.exists(), f"Downloaded file does not exist at {result_path}"
        assert result_path_obj.is_dir(), f"Expected directory at {result_path}"
        
        # Check if there are model files in the extracted directory
        model_files = list(result_path_obj.glob("*.usd")) + list(result_path_obj.glob("*.usda")) + list(result_path_obj.glob("*.usdc")) + list(result_path_obj.glob("*.glb"))
        assert len(model_files) > 0, f"No model files found in {result_path}"
        print(f"Verified: Model file from Chinese text successfully downloaded and extracted to {result_path}")
        
        # Test image generation
        image_url = "https://example.com/sample-image.jpg"
        task_id = meshy.generate_3d_from_image(image_url)
        assert task_id, "Task ID should be returned"
        print(f"Image generation test passed, task ID: {task_id}")
        result_path = meshy.monitor_task_status(task_id, "image-to-3d")
        result_path_obj = Path(result_path)
        assert result_path_obj.exists(), f"Downloaded file does not exist at {result_path}"
        assert result_path_obj.is_dir(), f"Expected directory at {result_path}"
        
        # Check if there are model files in the extracted directory
        model_files = list(result_path_obj.glob("*.usd")) + list(result_path_obj.glob("*.usda")) + list(result_path_obj.glob("*.usdc")) + list(result_path_obj.glob("*.glb"))
        assert len(model_files) > 0, f"No model files found in {result_path}"
        print(f"Verified: Model file successfully downloaded and extracted to {result_path}")
        
        print("All tests passed!")
        
    except Exception as e:
        print(f"Test failed: {str(e)}")


if __name__ == "__main__":
    

    asyncio.run(test_async())
    # task = asyncio.create_task(test_async())
    test()
    
    
    # Schedule the test to run in the background
    # task = asyncio.create_task(run_test_in_background())
    # task.add_done_callback(lambda _: print(f"Test completed: {result_path}"))

    # print("Test scheduled to run in background")
    # while result_path is None:
    #     sleep(1)
    # print(f"Async test completed, result path: {result_path}")
    
    
