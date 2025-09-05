# Use a lightweight Python image
FROM python:3.10-slim

# Create working directory
WORKDIR /workspace

# Copy and install Python dependencies
COPY env.yaml /workspace/env.yaml
# Install Python dependencies directly using pip for simplicity
RUN pip install --no-cache-dir numpy opencv-python matplotlib torch tf-transformations

# Copy the rest of the project files
COPY . /workspace

# Ensure the run script is executable
RUN chmod +x run.sh

# Default command executes the run script
ENTRYPOINT ["./run.sh"]
