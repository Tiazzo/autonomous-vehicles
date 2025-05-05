# split into test and train set
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
from sklearn.model_selection import learning_curve
import numpy as np
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
from sklearn.ensemble import RandomForestClassifier
from sklearn.svm import LinearSVC, SVC
import pandas as pd
import os
from pathlib import Path
from sklearn.metrics import precision_score, recall_score, f1_score
from sklearn.model_selection import GridSearchCV,RandomizedSearchCV
import joblib
from sklearn.preprocessing import StandardScaler,RobustScaler

folder_path = Path("./update_data")
all_files = folder_path.glob("*.csv")
df = pd.concat((pd.read_csv(file) for file in all_files), ignore_index=True)
#df = pd.read_csv('./update_data/update_trajectories-0515-0530.csv')

# From meeting with Marco
# cap to minimal distance between cars
# cap velocity to be higher than ego car v_E + 10?
# cap to 0 velocity if behind or v_E - 10?
# minmum ditance that does not affect the manouver find max value 
# test with hard cap and without hard cap for security and 
# Drop rows where mark = 0
# maybe the velocity of the car should also influence the "gap" between the non-existent car
df = df[df['mark'] != 0]
df['mark'] = df['mark'].replace(1, 0)
df['mark'] = df['mark'].replace(2, 1)

# Mean Space Headway: 59.03083733699383
# Max Space Headway: 378.75
df['delta_v_TR'] = df['v_E'] - df['v_TR']
df['delta_v_TP'] = df['v_E'] - df['v_TP']
#df['TTC_TR'] = df['G_TR'] / (df['v_E'] - df['v_TR'] + 1e-5)  # Prevent division by zero
columns = list(df.columns)
#put mark at the end of the dataframe
df['mark']=df.pop('mark')
print(df.head(1))
# save csv file
df.to_csv('./temp.csv', index=False)

df = df.drop(columns=['Vehicle_ID', 'Frame_ID','v_Class','Space_Hdwy','Time_Hdwy', 'Total_Frames','Lane_ID', 'Global_Time', 'Local_X', 'Preceeding', 'Local_Y',  'Global_X'  , 'Global_Y'])
# Save temp file
df.to_csv('./temp_clean.csv', index=False)
# Split data into training and test sets
x = df.iloc[:, :-1]
y = df.iloc[:, -1]

print(len(df))

## Split into train/test set
X_train, X_test, y_train, y_test = train_test_split(x, y, test_size=0.30, random_state=0) # Small test set as we have a large dataset

print(len(X_test))
print(len(X_train))

scaler = StandardScaler()
scaler.fit(X_train)
X_train = scaler.fit_transform(X_train)
X_test = scaler.transform(X_test)
# Check the mean and std of each feature after scaling
scaled_df = pd.DataFrame(X_train, columns=x.columns)
print("Means after scaling:\n", np.round(scaled_df.mean(), 5))
print("Standard deviations after scaling:\n", np.round(scaled_df.std(), 5))


# Define Model

css = np.arange(1,4,0.01)
acc = []
prec = []
rec = []
f = []
for i in css: 
    model = SVC(C=i,kernel='rbf', random_state=42, class_weight='balanced')
    model.fit(X_train, y_train)
    predictions = model.predict(X_test)
    accuracy = model.score(X_test, y_test) 
    acc.append(accuracy)
    
    # Calculate precision, recall, and F1-score
    y_true = y_test
    y_pred = predictions
    precision = precision_score(y_true, y_pred)
    recall = recall_score(y_true, y_pred)
    f1 = f1_score(y_true, y_pred)
    prec.append(precision)
    rec.append(recall)
    f.append(f1)
    

plt.xlabel('C')
plt.ylabel('Score')
plt.plot(css, acc,label="Accuracy")
# plt.scatter(css, acc)
plt.plot(css, prec,label="Precision")
# plt.scatter(css, prec)
plt.plot(css, rec,label="Recall")
# plt.scatter(css, rec)
plt.plot(css, f,label="F1")
# plt.scatter(css, f)
plt.legend()
plt.show()



# Evaluate the predictions

print("Accuracy of SVM:", np.max(acc), np.argmax(acc))
print("Max precision:", np.max(prec), np.argmax(prec))
print("Max recall:", np.max(rec), np.argmax(rec))
print("Max F1-score:", np.max(f), np.argmax(f))
print("C value for best accuracy:", css[np.argmax(acc)])


# Best model: 
model = SVC(C=css[np.argmax(acc)],kernel='rbf', random_state=42, class_weight='balanced')
model.fit(X_train, y_train)
y_pred = model.predict(X_test)
cm = confusion_matrix(y_test, y_pred)

# Display confusion matrix
disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=model.classes_)
disp.plot(cmap='Blues', colorbar=True)
plt.title("Confusion Matrix")
plt.show()

# Save the model and scaler
joblib.dump(model, 'svm_model.pkl')
joblib.dump(scaler, 'scaler.pkl')


