document.getElementById('user-input').addEventListener('keypress', function(event) {
    if (event.key === 'Enter') {
        event.preventDefault(); 
        sendMessage();
    }
});

function sendMessage() {
    const userInput = document.getElementById('user-input').value;
    if (!userInput) return; 
    appendMessage(userInput, 'user');
    document.getElementById('user-input').value = '';  
   
    fetch('/get_response', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ message: userInput })
    })
    .then(response => response.json())
    .then(data => {
        const botMessage = data.response;
      
        displayMessageGradually(botMessage, 'bot');
    })
    .catch(err => console.log('Error:', err));
}

function appendMessage(message, sender) {
    const chatWindow = document.getElementById('chat-window');
    const messageElement = document.createElement('div');
    messageElement.classList.add('message', `${sender}-message`);
    messageElement.innerHTML = formatMessage(message);
    
    chatWindow.appendChild(messageElement);
    chatWindow.scrollTop = chatWindow.scrollHeight;
}

function displayMessageGradually(message, sender) {
    const chatWindow = document.getElementById('chat-window');
    const messageElement = document.createElement('div');
    messageElement.classList.add('message', `${sender}-message`);
    messageElement.innerHTML = ''; 

    chatWindow.appendChild(messageElement);
    chatWindow.scrollTop = chatWindow.scrollHeight;

    let index = 0;
    const interval = setInterval(() => {
        messageElement.innerHTML += message.charAt(index);
        index++;

        if (index === message.length) {
            clearInterval(interval); 
        }
    }, 20); 
}

function formatMessage(message) {
    return `<div class="tree-node">${message}</div>`;
}
