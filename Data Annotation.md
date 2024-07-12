# Installing Label Studion
`python3 -m venv env`
`source env/bin/activate`
`sudo apt install python3.9-dev`
`python -m pip install label-studio`

# Run it 
`label-studio start`

# Create the xml file 
`<View>`

  `<Image name="img" value="$image"/>`
  
  `<PolygonLabels name="labels" toName="img">`
  
    `<Label value="Healthy Leaf" background="green"/>`
    
    `<Label value="Unhealthy Leaf" background="lightgreen"/>`
    
    `<Label value="Flower" background="white"/>`
    
  `</PolygonLabels>`
  
`</View>
`

# Create project folder
# Import images
# Start annotating
# Submit
# Export in coco format
