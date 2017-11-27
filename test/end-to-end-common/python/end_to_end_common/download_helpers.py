#!/usr/bin/env python

import hashlib
import os
import time
import urllib

def download_file(download_path, local_path, num_of_tries = 3):
  tries_so_far = 0
  while True:
    try:
      urllib.urlretrieve(download_path, local_path)
      return True
    except Exception as e:
      if tries_so_far >= num_of_tries:
        print "Couldn't download file from", download_path, "after", \
            num_of_tries, "tries."
        print "Last error message:", e
        return False

      tries_so_far += 1
      # Sleep for a while in hope that the network issue resolves itself.
      SLEEP_TIME_SECONDS = 0.01
      time.sleep(SLEEP_TIME_SECONDS)


def download_dataset(download_url, local_file_name):
  need_to_download_dataset = True
  if os.path.isfile(local_file_name):
    if download_file(download_url + ".sha256", local_file_name + ".sha256"):
      existing_file_sha256 = \
          hashlib.sha256(open(local_file_name, 'rb').read()).hexdigest()
      sha256_file = open(local_file_name + ".sha256", "r")
      file_to_download_sha256 = sha256_file. \
          readline()[0:len(existing_file_sha256)]

      if existing_file_sha256 == file_to_download_sha256:
        need_to_download_dataset = False
        print "Skipping download of the dataset as it already exists " \
              "under", local_file_name, "."
    else:
      print "Failed to download sha256 file.", \
          "Dataset needs to be downloaded again."


  if need_to_download_dataset:
    print "Downloading dataset from", download_url, "."
    if not download_file(download_url, local_file_name):
      print "Download of the dataset failed!"
      assert False
    print "Download complete!"

