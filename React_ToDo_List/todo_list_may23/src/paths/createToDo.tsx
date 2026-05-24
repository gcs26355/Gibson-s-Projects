import { Dispatch, SetStateAction } from "react"

interface ToDo {
    id: number;
    message: string;
    time_input: string;
    due_date: string;
}

export const CreateToDo = ( message: string, setTodoList: Dispatch<SetStateAction<ToDo[]>> ) => {
    const newTodo: ToDo = {
        id: Date.now(),
        message,
        time_input: new Date().toISOString(),
        due_date: "",
    };

    setTodoList((prev) => [...prev, newTodo]);
};