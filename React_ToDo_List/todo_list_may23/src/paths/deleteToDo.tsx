import { Dispatch, SetStateAction } from "react"

interface ToDo {
    id: number;
    message: string;
    time_input: string;
    due_date: string;
}

export const DeleteToDo = (messageID: number, setTodoList: Dispatch<SetStateAction<ToDo[]>>) => {
    setTodoList((prev) => prev.filter((todo) => todo.id !== messageID));
}