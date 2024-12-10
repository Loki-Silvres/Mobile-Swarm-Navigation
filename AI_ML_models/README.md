# AI/ML Models  

This folder contains two AI/ML models that were developed and used in our project:  

1. **Instance Segmentation Model**: YOLOv8-seg  
2. **Chatbot**: Gemini 1.5 Flash  

## Folder Structure  

```
├── chatbot  
│   ├── app.py  
│   ├── static/  
│   │   ├── css/  
│   │   │   └── styles.css  
│   │   └── js/  
│   │       └── script.js  
│   └── templates/  
│       └── index.html  
├── Instance_Segmentation_Model_YOLOv8-seg  
│   ├── args.yaml  
│   ├── various metrics and visualization files (e.g., confusion matrices, PR curves, etc.)  
│   ├── results.csv  
│   ├── train_batch*.jpg (training batch samples)  
│   ├── val_batch*_labels.jpg & val_batch*_pred.jpg (validation labels and predictions)  
│   └── weights/  
│       ├── best.pt  
│       └── last.pt  
└── README.md  
```

---

### **1. Instance Segmentation Model (YOLOv8-seg)**  
- **Purpose**: Performs instance segmentation on images.  
- **Details**:  
  - Contains training results, metrics, and visualizations for performance monitoring.  
  - **Key Files**:  
    - `weights/`: Contains the model weights (`best.pt` and `last.pt`).  
    - Training and validation visualizations such as confusion matrices, PR curves, and labeled predictions.  
    - `results.csv`: Summary of the training outcomes.  

---

### **2. Chatbot (Gemini 1.5 Flash)**  
- **Purpose**: A conversational AI implemented using a Flask web application.  
- **Details**:  
  - **Zero-shot Deployment**: The chatbot was not fine-tuned on specific data, leveraging its pre-trained knowledge for conversations.  
  - **Folder Breakdown**:  
    - `app.py`: The main script for running the chatbot Flask app.  
    - `static/`: Contains static files such as CSS (`styles.css`) and JavaScript (`script.js`).  
    - `templates/`: Contains HTML templates (e.g., `index.html`) for rendering the web application interface.  
